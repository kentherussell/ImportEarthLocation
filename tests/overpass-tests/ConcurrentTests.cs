using System.Collections.Concurrent;
using System.Net;
using System.Reflection;
using System.Text.RegularExpressions;
using ImportEarthLocation.Core;
using ImportEarthLocation.Data;
using ImportEarthLocation.Geo;

static class Program
{
    static long ticks;
    static void Check(bool value,string message) { if(!value) throw new Exception(message); }
    static Task Delay(TimeSpan time,CancellationToken token) { token.ThrowIfCancellationRequested(); Interlocked.Add(ref ticks,time.Ticks); return Task.CompletedTask; }
    static DateTimeOffset Now()=>DateTimeOffset.UnixEpoch.AddTicks(Interlocked.Read(ref ticks));
    static HttpResponseMessage Good(int tile)=>new(HttpStatusCode.OK){Content=new StringContent($"<osm version='0.6'><node id='{tile+1}' lat='0' lon='0'/></osm>")};
    static HttpResponseMessage Bad(HttpStatusCode status,string body)=>new(status){Content=new StringContent(body)};
    static async Task Main()
    {
        var cache=new CacheStore(); var log=new ImportLog();
        var handler=new Handler((tile,attempt,total)=>Good(tile));
        var client=new OverpassClient(cache,log,new HttpClient(handler),Delay,Now);
        var result=await client.Download16Async(default,false,default);
        Check(handler.Peak==3 && handler.Duplicated==0,"Concurrency bound or duplicate request violation");
        Check(result.Select((bytes,i)=>System.Text.Encoding.UTF8.GetString(bytes).Contains($"id='{i+1}'")).All(x=>x),"Output not in geographic tile order");
        Check(cache.Values.Count==16,"Cache incomplete");
        await client.Download16Async(default,false,default); Check(handler.Total==16,"Cache not resumed");
        handler=new Handler((tile,attempt,total)=>Bad(HttpStatusCode.GatewayTimeout,"<html>Server overloaded\nplease retry</html>"));
        log=new ImportLog(); client=new OverpassClient(new CacheStore(),log,new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,default); throw new Exception("Expected exhaustion"); }
        catch(HttpRequestException e) { Check(e.Message.Contains("18 attempts total"),"Failure budget message missing"); }
        Check(handler.Counts.GroupBy(pair=>pair.Key.tile).Any(group=>group.Count()==3 && group.All(pair=>pair.Value==6)),"Six attempts per provider not honored");
        Check(handler.Counts.Keys.All(key=>key.tile<3),"Continued tiles after exhausted geographic coverage");
        Check(log.Lines.Any(x=>x.Contains("HTTP 504") && x.Contains("Server overloaded please retry")),"Server details missing");
        handler=new Handler((tile,attempt,total)=>total<=2?Bad((HttpStatusCode)429,"rate limited"):Good(tile));
        log=new ImportLog(); client=new OverpassClient(new CacheStore(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default);
        Check((int)typeof(OverpassClient).GetField("_concurrency",BindingFlags.Instance|BindingFlags.NonPublic)!.GetValue(client)! == 1,"429 reduction failed");
        Check(log.Lines.Any(x=>x.Contains("concurrency to 2"))&&log.Lines.Any(x=>x.Contains("concurrency to 1")),"Reduction not logged");
        handler=new Handler((tile,attempt,total)=>tile==0&&attempt==1?Bad(HttpStatusCode.OK,"<osm version='0.6'><remark>runtime error: timeout</remark></osm>"):Good(tile));
        log=new ImportLog(); client=new OverpassClient(new CacheStore(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default); Check(log.Lines.Any(x=>x.Contains("invalid/incomplete")),"Partial XML accepted");
        handler=new Handler((tile,attempt,total)=>tile==0&&attempt==1?throw new TaskCanceledException():Good(tile));
        log=new ImportLog(); client=new OverpassClient(new CacheStore(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default); Check(log.Lines.Any(x=>x.Contains("HTTP client timed out")),"Timeout not retried");
        using var cancelled=new CancellationTokenSource(); cancelled.Cancel();
        handler=new Handler((tile,attempt,total)=>Good(tile)); client=new OverpassClient(new CacheStore(),new ImportLog(),new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,cancelled.Token); throw new Exception("Cancellation lost"); } catch(OperationCanceledException){}
        Check(handler.Total==0,"Cancelled download sent requests");
        handler=new Handler((tile,attempt,total)=>Bad(HttpStatusCode.BadRequest,"query error"));
        client=new OverpassClient(new CacheStore(),new ImportLog(),new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,default); throw new Exception("Permanent failure not rejected"); } catch(InvalidOperationException e){Check(e.Message.Contains("non-retryable"),"Wrong permanent failure");}
        Check(handler.Total<=3,"Permanent failure retried");
        Console.WriteLine("PASS: rolling concurrency≤3, no duplicate requests, deterministic indexed output, cache resume, 6/provider budget, incomplete-coverage abort, detailed errors, 429 reduction 3→2→1, HTTP-200 errors, timeout/cancellation/permanent failure");
    }
    sealed class Handler(Func<int,int,int,HttpResponseMessage> respond):HttpMessageHandler
    {
        int active; readonly ConcurrentDictionary<int,int> activeTiles=new();
        public int Peak,Total,Duplicated;
        public ConcurrentDictionary<(int tile,string host),int> Counts=new();
        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request,CancellationToken token)
        {
            var body=Uri.UnescapeDataString((await request.Content!.ReadAsStringAsync(token)).Substring(5).Replace("+"," "));
            var tile=int.Parse(Regex.Matches(body,@"\(([-0-9.,]+)\)")[0].Groups[1].Value.Split(',')[1]);
            var attempt=Counts.AddOrUpdate((tile,request.RequestUri!.Host),1,(_,old)=>old+1);
            var total=Interlocked.Increment(ref Total); var concurrent=Interlocked.Increment(ref active);
            if(activeTiles.AddOrUpdate(tile,1,(_,old)=>old+1)>1) Interlocked.Increment(ref Duplicated);
            lock(this) Peak=Math.Max(Peak,concurrent);
            try { await Task.Delay(15+(15-tile)%3,token); return respond(tile,attempt,total); }
            finally { Interlocked.Decrement(ref active); activeTiles.AddOrUpdate(tile,0,(_,old)=>old-1); }
        }
    }
}
namespace ImportEarthLocation.Core
{
    public sealed class ImportLog { public ConcurrentQueue<string> Lines=new(); public void Info(string x)=>Lines.Enqueue(x); public void Warn(string x)=>Lines.Enqueue(x); public void Count(string x,int count=1){} public void Time(string x,double seconds){} }
}
namespace ImportEarthLocation.Geo
{
    public struct GeoBounds { public double South,West,North,East; }
    public static class GeoMath { public static List<GeoBounds> Split4x4(GeoBounds bounds)=>Enumerable.Range(0,16).Select(i=>new GeoBounds{West=i,East=i+1,North=1}).ToList(); }
}
namespace ImportEarthLocation.Data
{
    public static class PublicHttpClient { public static HttpClient Instance=new(); }
    public sealed class CacheStore
    {
        public ConcurrentDictionary<string,byte[]> Values=new();
        public Task<byte[]?> ReadValidAsync(string group,string key,string extension,Func<byte[],bool> valid,CancellationToken token)=>Task.FromResult(Values.TryGetValue(key,out var bytes)&&valid(bytes)?bytes:null);
        public Task WriteAsync(string group,string key,string extension,byte[] bytes,CancellationToken token){token.ThrowIfCancellationRequested();Values[key]=bytes;return Task.CompletedTask;}
    }
}
