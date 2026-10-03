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
        var endpoints = new[] { "https://photon.komoot.io/api/?limit=6&q=" + encoded,
            "https://geocoding-api.open-meteo.com/v1/search?count=6&language=en&name=" + encoded };
        for (var provider = 0; provider < endpoints.Length; provider++)
        {
            try
            {
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeout.CancelAfter(TimeSpan.FromSeconds(10));
                using var response = await PublicHttpClient.Instance.GetAsync(endpoints[provider], timeout.Token).ConfigureAwait(false);
                if (!response.IsSuccessStatusCode) continue; // No hammering a rate-limited provider.
                var root = JObject.Parse(await response.Content.ReadAsStringAsync().ConfigureAwait(false));
                var results = new JArray();
                foreach (var item in (root[provider == 0 ? "features" : "results"] as JArray ?? new JArray()).Take(6))
                {
                    var p = provider == 0 ? item["properties"]! : item;
                    var latitude = provider == 0 ? (double?)item["geometry"]?["coordinates"]?[1] : (double?)item["latitude"];
                    var longitude = provider == 0 ? (double?)item["geometry"]?["coordinates"]?[0] : (double?)item["longitude"];
                    if (!latitude.HasValue || !longitude.HasValue || !Valid(latitude.Value, longitude.Value)) continue;
                    var label = string.Join(", ", new[] { (string?)p["name"], (string?)p[provider == 0 ? "state" : "admin1"], (string?)p["country"] }.Where(s => !string.IsNullOrWhiteSpace(s)).Distinct());
                    if (label.Length == 0) continue;
                    results.Add(new JObject { ["label"] = label, ["latitude"] = latitude, ["longitude"] = longitude });
                }
                if (results.Count > 0) return new JObject { ["results"] = results, ["message"] = "Select a result.", ["source"] = provider == 0 ? "Photon / OpenStreetMap" : "Open-Meteo / GeoNames" }.ToString(Newtonsoft.Json.Formatting.None);
            }
            catch (Exception) when (!token.IsCancellationRequested) { /* Try the independent fallback once. */ }
            token.ThrowIfCancellationRequested();
        }
        return "{\"results\":[],\"message\":\"Place search unavailable. Pan the map or enter coordinates manually.\"}";
    }
}
