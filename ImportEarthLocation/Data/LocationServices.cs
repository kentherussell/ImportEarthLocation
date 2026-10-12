using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Data;

/// <summary>Interactive search only. No importer downloads or map mutation.</summary>
public static class LocationServices
{
    public static async Task<string> TilesAsync(string payload, CancellationToken token)
    {
        var input = JObject.Parse(payload);
        var revision = (int?)input["revision"] ?? 0;
        var keys = ((JArray?)input["keys"] ?? new JArray()).Values<string>().Distinct().Take(16).ToArray();
        var tiles = new JObject();
        using var slots = new SemaphoreSlim(4);
        var cache = new CacheStore();
        await Task.WhenAll(keys.Select(async key =>
        {
            if (key == null) return;
            var parts = key.Split('/');
            if (parts.Length != 3 || !int.TryParse(parts[0], out var z) || !int.TryParse(parts[1], out var x) || !int.TryParse(parts[2], out var y) || z < 1 || z > 16 || x < 0 || y < 0 || x >= (1 << z) || y >= (1 << z)) return;
            await slots.WaitAsync(token).ConfigureAwait(false);
            try
            {
                var cacheKey = "display-osm-v1:" + key;
                var bytes = await cache.ReadValidAsync("display", cacheKey, "png", ValidPng, token).ConfigureAwait(false);
                if (bytes == null)
                {
                    using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                    timeout.CancelAfter(TimeSpan.FromSeconds(12));
                    using var response = await PublicHttpClient.Instance.GetAsync("https://tile.openstreetmap.org/" + key + ".png", timeout.Token).ConfigureAwait(false);
                    response.EnsureSuccessStatusCode();
                    bytes = await response.Content.ReadAsByteArrayAsync().ConfigureAwait(false);
                    if (!ValidPng(bytes)) return;
                    await cache.WriteAsync("display", cacheKey, "png", bytes, token).ConfigureAwait(false); // No prefetch/forced refresh; retained >=7 days.
                }
                lock (tiles) tiles[key] = "data:image/png;base64," + Convert.ToBase64String(bytes);
            }
            catch (Exception) when (!token.IsCancellationRequested) { /* Coordinate fallback remains available. */ }
            finally { slots.Release(); }
        })).ConfigureAwait(false);
        return new JObject { ["revision"] = revision, ["tiles"] = tiles, ["error"] = tiles.Count != keys.Length }.ToString(Newtonsoft.Json.Formatting.None);
    }

    private static bool ValidPng(byte[] bytes) => bytes.Length >= 8 && bytes.Length <= 300_000 && bytes[0] == 137 && bytes[1] == 80 && bytes[2] == 78 && bytes[3] == 71;

    public static bool Valid(double latitude, double longitude) =>
        !double.IsNaN(latitude) && !double.IsInfinity(latitude) && Math.Abs(latitude) <= 90 &&
        !double.IsNaN(longitude) && !double.IsInfinity(longitude) && Math.Abs(longitude) <= 180;

    public static async Task<string> SearchAsync(string query, CancellationToken token)
    {
        query = query.Trim();
        if (query.Length < 2 || query.Length > 160) return "{\"results\":[],\"message\":\"Enter a place name (2–160 characters).\"}";
        var encoded = Uri.EscapeDataString(query);
        var endpoints = new[] { "https://photon.komoot.io/api/?limit=20&lang=en&q=" + encoded,
            "https://geocoding-api.open-meteo.com/v1/search?count=20&language=en&name=" + encoded };
        async Task<List<JObject>> Fetch(int provider)
        {
            var results=new List<JObject>();
            try
            {
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeout.CancelAfter(TimeSpan.FromSeconds(10));
                using var response = await PublicHttpClient.Instance.GetAsync(endpoints[provider], timeout.Token).ConfigureAwait(false);
                if (!response.IsSuccessStatusCode) return results;
                var root = JObject.Parse(await response.Content.ReadAsStringAsync().ConfigureAwait(false));
                foreach (var item in (root[provider == 0 ? "features" : "results"] as JArray ?? new JArray()).Take(20))
                {
                    var p = provider == 0 ? item["properties"]! : item;
                    var latitude = provider == 0 ? (double?)item["geometry"]?["coordinates"]?[1] : (double?)item["latitude"];
                    var longitude = provider == 0 ? (double?)item["geometry"]?["coordinates"]?[0] : (double?)item["longitude"];
                    if (!latitude.HasValue || !longitude.HasValue || !Valid(latitude.Value, longitude.Value)) continue;
                    var name=(string?)p["name:en"]??(string?)p["name"]??"";
                    var label = string.Join(", ", new[] { name, (string?)p[provider == 0 ? "state" : "admin1"], (string?)p["country"] }.Where(s => !string.IsNullOrWhiteSpace(s)).Distinct());
                    if (label.Length == 0) continue;
                    results.Add(new JObject { ["label"] = label, ["latitude"] = latitude, ["longitude"] = longitude,
                        ["exactName"]=string.Equals(name,query,StringComparison.OrdinalIgnoreCase),["population"]=(long?)p["population"]??0,
                        ["city"]=(string?)p["osm_value"]=="city"||(string?)p["feature_code"]=="PPLC"||(string?)p["feature_code"]=="PPLA",
                        ["providerRank"]=results.Count,["source"]=provider==0?"Photon / OpenStreetMap":"Open-Meteo / GeoNames" });
                }
            }
            catch (Exception) when (!token.IsCancellationRequested) { /* Try the independent fallback once. */ }
            token.ThrowIfCancellationRequested();
            return results;
        }
        var providers=await Task.WhenAll(Fetch(0),Fetch(1)).ConfigureAwait(false);
        var merged=providers.SelectMany(p=>p).OrderByDescending(p=>(bool?)p["exactName"]??false)
            .ThenByDescending(p=>(long?)p["population"]??0).ThenByDescending(p=>(bool?)p["city"]??false)
            .ThenBy(p=>(int?)p["providerRank"]??0).ToArray();
        var selected=new List<JObject>();
        foreach(var item in merged)
        {
            if(selected.Any(p=>Math.Abs((double)p["latitude"]!-(double)item["latitude"]!)<.025&&Math.Abs((double)p["longitude"]!-(double)item["longitude"]!)<.025))continue;
            selected.Add(item);if(selected.Count>=12)break;
        }
        if(selected.Count>0)return new JObject{["results"]=new JArray(selected),["message"]="Select a result.",["source"]="English names; exact match, population and city relevance"}.ToString(Newtonsoft.Json.Formatting.None);
        return "{\"results\":[],\"message\":\"Place search unavailable. Pan the map or enter coordinates manually.\"}";
    }
}
