using System;
using System.Net.Http;
using System.Net;
using System.Threading;

namespace ImportEarthLocation.Data;

public static class PublicHttpClient
{
    private static readonly Lazy<HttpClient> LazyClient = new(() =>
    {
        var handler = new HttpClientHandler { AutomaticDecompression = DecompressionMethods.GZip | DecompressionMethods.Deflate };
        var client = new HttpClient(handler) { Timeout = TimeSpan.FromSeconds(90) };
        client.DefaultRequestHeaders.UserAgent.ParseAdd($"{Mod.Id}/{Mod.Version} (Cities Skylines II editor mod)");
        return client;
    });

    public static HttpClient Instance => LazyClient.Value;

    public static async System.Threading.Tasks.Task DelayForRetryAsync(int attempt, CancellationToken token)
    {
        var seconds = Math.Min(20, (int)Math.Pow(2, attempt));
        await System.Threading.Tasks.Task.Delay(TimeSpan.FromSeconds(seconds), token).ConfigureAwait(false);
    }
}
