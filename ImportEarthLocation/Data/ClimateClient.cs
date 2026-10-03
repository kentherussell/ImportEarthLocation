using System;
using System.Globalization;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Data;

public static class ClimateClient
{
    private static readonly string[] Months = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    public static async Task<ClimateNormals?> LoadAsync(ImportRequest request, ImportLog log, CancellationToken token)
    {
        try
        {
            var url = "https://power.larc.nasa.gov/api/temporal/climatology/point?parameters=T2M,PRECTOTCORR,WS10M,WD10M&community=SB&format=JSON&start=2001&end=2020&latitude=" +
                request.Latitude.ToString("R", CultureInfo.InvariantCulture) + "&longitude=" + request.Longitude.ToString("R", CultureInfo.InvariantCulture);
            var cache = new CacheStore();
            var bytes = await cache.ReadValidAsync("climate", url, "json", b => { try { Parse(b); return true; } catch { return false; } }, token).ConfigureAwait(false);
            if (bytes == null)
            {
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeout.CancelAfter(TimeSpan.FromSeconds(20));
                using var response = await PublicHttpClient.Instance.GetAsync(url, timeout.Token).ConfigureAwait(false);
                response.EnsureSuccessStatusCode();
                bytes = await response.Content.ReadAsByteArrayAsync().ConfigureAwait(false);
                Parse(bytes);
                await cache.WriteAsync("climate", url, "json", bytes, token).ConfigureAwait(false);
            }
            var normals = Parse(bytes);
            log.Info("Climate normals prepared: " + normals.SourceVersion + "; fixed 2001–2020 climatology, not live weather. A cached location is usable offline.");
            return normals;
        }
        catch (Exception exception) when (!token.IsCancellationRequested)
        {
            log.Warn("Climate data unavailable; preserving existing/default map climate: " + exception.Message);
            return null;
        }
    }
    public static ClimateNormals Parse(byte[] bytes)
    {
        if (bytes.Length > 100_000) throw new InvalidOperationException("Unexpectedly large climate response.");
        var root = JObject.Parse(Encoding.UTF8.GetString(bytes));
        var parameters = root["properties"]?["parameter"] ?? throw new InvalidOperationException("Missing climate normals.");
        float[] Read(string name, float min, float max)
        {
            var values = new float[12];
            for (var i = 0; i < 12; i++)
            {
                var value = (float?)parameters[name]?[Months[i]];
                if (!value.HasValue || float.IsNaN(value.Value) || float.IsInfinity(value.Value) || value < min || value > max)
                    throw new InvalidOperationException($"Invalid {name} climatology for {Months[i]}.");
                values[i] = value.Value;
            }
            return values;
        }
        var elevation = (float?)root["geometry"]?["coordinates"]?[2] ?? 0;
        if (float.IsNaN(elevation) || float.IsInfinity(elevation) || elevation < -500 || elevation > 9000) throw new InvalidOperationException("Invalid climate grid elevation.");
        var range = (string?)root["header"]?["range"] ?? "";
        if (!range.Contains("2001") || !range.Contains("2020")) throw new InvalidOperationException("Unexpected climatological period; cannot silently mix dataset versions.");
        return new ClimateNormals(Read("T2M", -90, 65), Read("PRECTOTCORR", 0, 100), Read("WS10M", 0, 60), Read("WD10M", 0, 360), elevation,
            "NASA POWER / MERRA-2, 2001–2020; API " + (string?)root["header"]?["api"]?["version"]);
    }
}
