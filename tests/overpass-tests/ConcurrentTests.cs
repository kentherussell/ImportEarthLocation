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
    static readonly string CacheRoot=Path.Combine(Path.GetTempPath(),"IEL-overpass-smoke-"+Guid.NewGuid().ToString("N"));
    static CacheStore NewCache()=>new(Path.Combine(CacheRoot,Guid.NewGuid().ToString("N")));
    static void Check(bool value,string message) { if(!value) throw new Exception(message); }
    static Task Delay(TimeSpan time,CancellationToken token) { token.ThrowIfCancellationRequested(); Interlocked.Add(ref ticks,time.Ticks); return Task.CompletedTask; }
    static DateTimeOffset Now()=>DateTimeOffset.UnixEpoch.AddTicks(Interlocked.Read(ref ticks));
    static HttpResponseMessage Good(int tile)=>new(HttpStatusCode.OK){Content=new StringContent($"<osm version='0.6'><node id='{tile+1}' lat='0' lon='0'/></osm>")};
    static HttpResponseMessage Bad(HttpStatusCode status,string body)=>new(status){Content=new StringContent(body)};
    static async Task Main(string[] args)
    {
        if(args.Contains("--live-queries")) { await LiveQueries(); return; }
        var cache=NewCache(); var log=new ImportLog();
        await CacheSmoke();
        var forestQuery=OverpassClient.BuildForestQuery(new GeoBounds{South=0,West=0,North=1,East=1});
        Check(forestQuery.Contains("^(tree|shrub)$")&&forestQuery.Contains("tree_row|wood|scrub|heath|wetland|beach")&&forestQuery.Contains("forest|orchard")&&forestQuery.Contains("(._;>>;)->.features;"),"Forest query omits mapped trees/rows, woodland or recursive member geometry");
        Check(forestQuery.Contains("node.features[\"natural\"~\"^(tree|shrub)$\"]->.trees;")&&forestQuery.Contains("(way.features;relation.features;.trees;);out body qt;")&&forestQuery.Contains("(node.features; - .trees;);out skel qt;"),"Forest output loses tree/shrub tags or strips member way/relation tags instead of supporting node tags");
        var networkQuery=OverpassClient.BuildNetworkQuery(default,RoadImportLevel.Primary,true);
        Check(networkQuery.Contains("(.ways;.movements;);out body qt;")&&networkQuery.EndsWith(".tagged out body qt;(.nodes; - .tagged;);out skel qt;")&&!networkQuery.Contains("out geom")&&!networkQuery.Contains(";-."),"Network output loses tagged nodes/movement geometry or emits invalid Overpass subtraction syntax");
        foreach(var painting in new[]{false,true})
        {
            var geographyQuery=OverpassClient.BuildGeographyQuery(default,painting);
            Check(geographyQuery.Contains("scrub")==painting&&geographyQuery.Contains("farmland")==painting&&geographyQuery.Contains("bare_rock")==painting,"Surface evidence not gated by painting preference");
            Check(geographyQuery.Contains("river|stream|riverbank|canal|drain|ditch"),"Disabling painting removed river/canal/drain hydrology");
            Check(geographyQuery.Contains("coastline|beach")&&geographyQuery.Contains("river|stream|riverbank")&&geographyQuery.Contains("reservoir")&&geographyQuery.Contains("quarry"),"Disabling painting removed hydrology or terrain protection");
            Check(geographyQuery.Contains("sand")==painting&&geographyQuery.Contains("grassland")==painting&&geographyQuery.Contains("meadow")==painting,"Painting-only feature selection ignores the surface preference");
            Check(geographyQuery.Contains("residential")==painting&&geographyQuery.Contains("paving_stones")==painting,"Desert non-sand protection missing or requested with painting disabled");
            Check(geographyQuery.EndsWith("(._;>>;)->.features;(way.features;relation.features;);out body qt;node.features;out skel qt;"),"Geography lost recursive members or member tags");
        }
        Check(OverpassProviders.Endpoints.Length==4&&new Uri(OverpassProviders.Endpoints[0]).Host=="overpass.nextgis.com","Temporary OSM primary not first");
        Check(OverpassProviders.BuildEndpoints(null).Length==3&&OverpassProviders.BuildEndpoints(null).All(e=>!e.Contains("nextgis")),"Removing the temporary setting does not restore public-only providers");
        var primaryPreference=new OverpassRequestScheduler();primaryPreference.RecordSuccess(2,Now(),1);
        Check(Enumerable.Range(0,16).All(tile=>primaryPreference.ProviderOrder(tile,Now())[0]==0),"Recent fallback success or tile rotation overrides temporary primary");
        var primaryHandler=new Handler((tile,attempt,total)=>Good(tile));
        PublicHttpClient.Instance=new HttpClient(primaryHandler);
        foreach(var category in new[]{"networks","forests","geography"})
        {
            var primaryLog=new ImportLog();
            await new OverpassClient(NewCache(),primaryLog,forestOnly:category=="forests",geography:category=="geography").DownloadMapAsync(default,false,default);
            Check(!primaryLog.Lines.Any(line=>line.Contains(new Uri(OverpassProviders.Endpoints[0]).AbsolutePath)),"Provider access key leaked into diagnostics");
        }
        Check(primaryHandler.Total==48&&primaryHandler.Counts.Keys.All(k=>k.host=="overpass.nextgis.com")&&primaryHandler.PeakPerProvider==1,"A new OSM request bypassed the primary while it was busy");
        var preference=new OverpassRequestScheduler(3,null);
        preference.RecordSuccess(2,Now(),1);
        Check(preference.ProviderOrder(0,Now())[0]==2,"Recent successful provider not preferred");
        preference.CoolDown(2,Now().AddMinutes(1));
        Check(preference.ProviderOrder(0,Now())[0]!=2,"Cooling provider preferred over available alternatives");
        preference.RecordFailure(0);
        Check(preference.ProviderOrder(0,Now())[0]==1,"Failed provider not deprioritized");
        Check(preference.ProviderOrder(0,Now()).OrderBy(x=>x).SequenceEqual(new[]{0,1,2}),"Preference dropped a retry provider");
        var readyScheduler=new OverpassRequestScheduler();readyScheduler.CoolDown(0,Now().AddMinutes(10));
        var selected=await readyScheduler.AcquireAnyAsync("networks",_=>true,0,Now,Delay,default).WaitAsync(TimeSpan.FromSeconds(2));
        Check(selected!=0,"Dispatcher waited for a cooling provider instead of choosing a ready alternative");readyScheduler.Release(selected);
        var fairScheduler=new OverpassRequestScheduler(OverpassProviders.Endpoints.Length,null);
        var slotA=await fairScheduler.AcquireAnyAsync("forests",_=>true,0,Now,Delay,default);
        var slotB=await fairScheduler.AcquireAnyAsync("forests",_=>true,1,Now,Delay,default);
        var forestWait=fairScheduler.AcquireAnyAsync("forests",_=>true,2,Now,Delay,default);
        var geographyWait=fairScheduler.AcquireAnyAsync("geography",_=>true,2,Now,Delay,default);
        fairScheduler.Release(slotA);
        var geographySlot=await geographyWait.WaitAsync(TimeSpan.FromSeconds(2));
        Check(!forestWait.IsCompleted,"Queued forests starved the geography category");
        fairScheduler.Release(geographySlot);fairScheduler.Release(slotB);
        fairScheduler.Release(await forestWait.WaitAsync(TimeSpan.FromSeconds(2)));
        var wholeCache=NewCache();var wholeHandler=new Handler((tile,attempt,total)=>Good(tile));
        var wholeClient=new OverpassClient(wholeCache,new ImportLog(),new HttpClient(wholeHandler),Delay,Now);
        Check((await wholeClient.DownloadMapAsync(default,false,default)).Count==16&&wholeHandler.Total==16,"Rollback did not restore tiled downloads");
        await wholeClient.DownloadMapAsync(default,false,default);Check(wholeHandler.Total==16,"Tiled cache not reused");
        var fallbackHandler=new Handler((tile,attempt,total)=>tile==-1?Bad(HttpStatusCode.OK,"<osm version='0.6'><remark>runtime error: Query timed out</remark></osm>"):Good(tile));
        var fallbackClient=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(fallbackHandler),Delay,Now);
        Check((await fallbackClient.DownloadMapAsync(default,false,default)).Count==16&&fallbackHandler.Total==16,"Rollback attempted a whole-map request");
        var resumeCache=NewCache();var tileBounds=GeoMath.Split4x4(default)[0];
        await resumeCache.WriteAsync("overpass",$"overpass-v4:networks:0:{OverpassClient.BuildNetworkQuery(tileBounds,RoadImportLevel.Primary,true)}","osm",System.Text.Encoding.UTF8.GetBytes("<osm version='0.6'><node id='1' lat='0' lon='0'/></osm>"),default);
        var resumeHandler=new Handler((tile,attempt,total)=>Good(tile));
        var resumeClient=new OverpassClient(resumeCache,new ImportLog(),new HttpClient(resumeHandler),Delay,Now);
        await resumeClient.DownloadMapAsync(default,false,default);
        Check(resumeHandler.Total==15&&resumeHandler.Counts.Keys.All(k=>k.tile>=1),"Partial tile cache did not skip the whole-map attempt/missing-only resume");
        var handler=new Handler((tile,attempt,total)=>Good(tile));
        var client=new OverpassClient(cache,log,new HttpClient(handler),Delay,Now);
        var result=await client.Download16Async(default,false,default);
        Check(handler.Peak==1 && handler.Duplicated==0,"Primary-first queue or duplicate request violation");
        Check(result.Select((bytes,i)=>System.Text.Encoding.UTF8.GetString(bytes).Contains($"id='{i+1}'")).All(x=>x),"Output not in geographic tile order");
        Check(Directory.GetFiles(Path.Combine(cache.Root,"overpass"),"*.osm").Length==16,"Cache incomplete");
        await client.Download16Async(default,false,default); Check(handler.Total==16,"Cache not resumed");
        var sharedScheduler=new OverpassRequestScheduler(OverpassProviders.Endpoints.Length,null);
        var overlap=new RequestBarrier(2);
        handler=new Handler((tile,attempt,total)=>Good(tile),overlap);
        var firstClient=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(handler),Delay,Now,sharedScheduler);
        var secondClient=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(handler),Delay,Now,sharedScheduler);
        var overlappingDownloads=Task.WhenAll(firstClient.Download16Async(default,false,default),secondClient.Download16Async(default,false,default));
        try
        {
            await overlap.Entered.WaitAsync(TimeSpan.FromSeconds(5));
            Check(handler.Peak==2&&handler.PeakPerProvider==1,"Held requests did not genuinely overlap within provider budgets");
        }
        finally { overlap.Release(); }
        await overlappingDownloads;
        Check(handler.Total==32&&handler.Peak==2,"Separate clients bypassed the shared HTTP budget");
        Check(handler.PeakPerProvider==1,"Separate clients sent overlapping requests to one provider");
        handler=new Handler((tile,attempt,total)=>Bad(HttpStatusCode.GatewayTimeout,"<html>Server overloaded\nplease retry</html>"));
        log=new ImportLog(); client=new OverpassClient(NewCache(),log,new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,default); throw new Exception("Expected exhaustion"); }
        catch(HttpRequestException e) { Check(e.Message.Contains("24 attempts total"),"Failure budget message missing"); }
        Check(handler.Counts.GroupBy(pair=>pair.Key.tile).Any(group=>group.Count()==4 && group.All(pair=>pair.Value==6)),"Six attempts per provider not honored");
        Check(handler.Counts.Keys.All(key=>key.tile<3),"Continued tiles after exhausted geographic coverage");
        Check(log.Lines.Any(x=>x.Contains("HTTP 504") && x.Contains("Server overloaded please retry")),"Server details missing");
        var partialCache=NewCache();
        var cachedKey=$"overpass-v4:networks:15:{OverpassClient.BuildNetworkQuery(GeoMath.Split4x4(default)[15],RoadImportLevel.Primary,true)}";
        await partialCache.WriteAsync("overpass",cachedKey,"osm",System.Text.Encoding.UTF8.GetBytes("<osm><node id='999' lat='0' lon='0'/></osm>"),default);
        var partialHandler=new Handler((tile,attempt,total)=>tile==0?Bad(HttpStatusCode.GatewayTimeout,"exhausted"):Good(tile));
        var partialLog=new ImportLog();var partialClient=new OverpassClient(partialCache,partialLog,new HttpClient(partialHandler),Delay,Now);
        var partial=await partialClient.DownloadMapAsync(default,false,default,allowPartial:true);
        Check(partial.Count>0&&partial.Any(b=>System.Text.Encoding.UTF8.GetString(b).Contains("999")),"Retry exhaustion discarded unread/healthy cache tiles");
        Check(partialLog.Lines.Any(x=>x.Contains("coverageComplete=false"))&&partialLog.Lines.Any(x=>x.Contains("Preparation continues")),"Partial coverage incorrectly reported as complete");
        partialHandler=new Handler((tile,attempt,total)=>Bad(HttpStatusCode.BadRequest,"bad query"));
        partialClient=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(partialHandler),Delay,Now);
        Check((await partialClient.DownloadMapAsync(default,false,default,allowPartial:true)).Count==0,"No OSM data still blocked generation");
        handler=new Handler((tile,attempt,total)=>total<=2?Bad((HttpStatusCode)429,"rate limited"):Good(tile));
        log=new ImportLog(); client=new OverpassClient(NewCache(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default);
        var scheduler=(OverpassRequestScheduler)typeof(OverpassClient).GetField("_scheduler",BindingFlags.Instance|BindingFlags.NonPublic)!.GetValue(client)!;
        Check(scheduler.Concurrency>=1&&scheduler.Concurrency<=2,"Shared rate limit left allowed bounds");
        Check(log.Lines.Any(x=>x.Contains("shared concurrency to 1")),"Shared 429 reduction not logged");
        handler=new Handler((tile,attempt,total)=>tile==0&&attempt==1?Bad(HttpStatusCode.OK,"<osm version='0.6'><remark>runtime error: timeout</remark></osm>"):Good(tile));
        log=new ImportLog(); client=new OverpassClient(NewCache(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default); Check(log.Lines.Any(x=>x.Contains("invalid/incomplete")),"Partial XML accepted");
        handler=new Handler((tile,attempt,total)=>tile==0&&attempt==1?throw new TaskCanceledException():Good(tile));
        log=new ImportLog(); client=new OverpassClient(NewCache(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default); Check(log.Lines.Any(x=>x.Contains("HTTP client timed out")),"Timeout not retried");
        using var cancelled=new CancellationTokenSource(); cancelled.Cancel();
        handler=new Handler((tile,attempt,total)=>Good(tile)); client=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,cancelled.Token); throw new Exception("Cancellation lost"); } catch(OperationCanceledException){}
        Check(handler.Total==0,"Cancelled download sent requests");
        handler=new Handler((tile,attempt,total)=>Bad(HttpStatusCode.BadRequest,"query error"));
        client=new OverpassClient(NewCache(),new ImportLog(),new HttpClient(handler),Delay,Now);
        try { await client.Download16Async(default,false,default); throw new Exception("Permanent failure not rejected"); } catch(InvalidOperationException e){Check(e.Message.Contains("non-retryable"),"Wrong permanent failure");}
        Check(handler.Total<=3,"Permanent failure retried");
        foreach(var rejection in new[]{HttpStatusCode.Forbidden,HttpStatusCode.NotFound,HttpStatusCode.TooManyRequests})
        {
            var providerHandler=new Handler((tile,attempt,total)=>
            {
                if(total!=1)return Good(tile);
                var response=Bad(rejection,"provider temporarily unavailable");
                if(rejection==HttpStatusCode.TooManyRequests)response.Headers.RetryAfter=new System.Net.Http.Headers.RetryConditionHeaderValue(TimeSpan.FromHours(1));
                return response;
            });
            log=new ImportLog();client=new OverpassClient(NewCache(),log,new HttpClient(providerHandler),Delay,Now);
            Check((await client.Download16Async(default,false,default)).Count==16,"Provider rejection discarded complete coverage instead of using configured fallback");
            Check(providerHandler.Counts.Keys.Any(k=>k.host!="overpass.nextgis.com"),"Provider rejection did not use existing fallback");
            Check(log.Lines.Any(l=>l.Contains("existingFallbacksAllowed=true")),"Provider fallback not diagnosed");
        }
        handler=new Handler((tile,attempt,total)=>total==1?throw new HttpRequestException("Failed "+OverpassProviders.Endpoints[0]):Good(tile));
        log=new ImportLog();client=new OverpassClient(NewCache(),log,new HttpClient(handler),Delay,Now);
        await client.Download16Async(default,false,default);
        Check(!log.Lines.Any(l=>l.Contains(new Uri(OverpassProviders.Endpoints[0]).AbsolutePath)),"Access key leaked from HTTP exception details");
        Check(log.Lines.Any(l=>l.Contains("OSM REQUEST")&&l.Contains("queryId=")),"Request fingerprint missing");
        // Keep a small real-time test of the unchanged default production wait
        // paths. Most fake responses yield instead of sleeping for every request.
        var realHandler=new Handler((tile,attempt,total)=>Good(tile),realTime:true);
        PublicHttpClient.Instance=new HttpClient(realHandler);
        Check((await new OverpassClient(NewCache(),new ImportLog()).Download16Async(default,false,default)).Count==16&&realHandler.Total==16&&realHandler.PeakPerProvider==1,
            "Default production scheduler/clock failed real-time asynchronous smoke");
        Console.WriteLine("PASS: explicit held-request overlap and default real-time production scheduler; mock latency removed, retry/provider limits unchanged.");
        Console.WriteLine("PASS: shared multi-client concurrency≤2, one request/provider, no duplicate requests, deterministic indexed output, cache resume, 6/provider budget, opt-in partial/no-data recovery and unread cache retention, strict incomplete-coverage abort, detailed errors, shared 429 reduction 2→1, HTTP-200 errors, timeout/cancellation/permanent failure");
    }
    static async Task CacheSmoke()
    {
        var cache=NewCache();var bytes=System.Text.Encoding.UTF8.GetBytes("<osm version='0.6'><node id='1' lat='0' lon='0'/></osm>");
        await Task.WhenAll(Enumerable.Range(0,12).Select(_=>cache.WriteAsync("overpass","shared-key","osm",bytes,default)));
        Check((await cache.ReadValidAsync("overpass","shared-key","osm",b=>b.SequenceEqual(bytes),default))!.SequenceEqual(bytes),"Concurrent cache publication corrupted payload");
        Check(Directory.GetFiles(Path.Combine(cache.Root,"overpass"),"*.download").Length==0,"Cache left staging files");
        await cache.WriteAsync("overpass","invalid","osm",new byte[]{1,2,3},default);
        Check(await cache.ReadValidAsync("overpass","invalid","osm",_=>false,default)==null,"Invalid cached payload accepted");
        Check(Directory.GetFiles(Path.Combine(cache.Root,"overpass"),"*.invalid-*").Length==1,"Invalid payload not retained for diagnosis");
        var blockedRoot=Path.Combine(CacheRoot,"blocked-cache");File.WriteAllText(blockedRoot,"not a directory");
        var log=new ImportLog();var blocked=new CacheStore(blockedRoot,log);
        Check(await blocked.ReadValidAsync("overpass","key","osm",_=>true,default)==null,"Blocked cache read did not become miss");
        await blocked.WriteAsync("overpass","key","osm",bytes,default);
        var handler=new Handler((tile,attempt,total)=>Good(tile));
        Check((await new OverpassClient(blocked,log,new HttpClient(handler),Delay,Now).Download16Async(default,false,default)).Count==16,"Cache I/O failure discarded validated network download");
        Check(log.Lines.Any(l=>l.Contains("cacheOptional=true")),"Cache failure not diagnosed");
        using var cancelled=new CancellationTokenSource();cancelled.Cancel();
        try{await cache.WriteAsync("overpass","cancelled","osm",bytes,cancelled.Token);throw new Exception("Cache swallowed cancellation");}catch(OperationCanceledException){}
        Console.WriteLine("PASS cache: actual production cache; concurrent atomic writes, invalid quarantine, disk failure/download preservation, cancellation.");
    }
    static async Task LiveQueries()
    {
        // Explicit opt-in only. Exercise the real builders on tiny Marseille
        // bounds, sequentially; never print the temporary provider access URL.
        var bounds=new GeoBounds{South=43.295,West=5.369,North=43.296,East=5.370};
        var queries=new List<(string name,string query)>();
        var network=OverpassClient.BuildNetworkQuery(bounds,RoadImportLevel.Secondary,true);
        queries.Add(("historical-invalid-subtraction",network.Replace("; - .tagged;",";-.tagged;")));
        foreach(var level in Enum.GetValues<RoadImportLevel>())
            foreach(var rail in new[]{false,true})
                queries.Add(($"networks-{level}-rail-{rail}",OverpassClient.BuildNetworkQuery(bounds,level,rail)));
        queries.Add(("forests",OverpassClient.BuildForestQuery(bounds)));
        foreach(var paint in new[]{false,true})queries.Add(($"geography-paint-{paint}",OverpassClient.BuildGeographyQuery(bounds,paint)));
        queries.Add(("dependencies",OverpassClient.BuildDependencyQuery(new long[]{1},new long[]{1})));
        queries.Add(("network-nonempty-closure",OverpassClient.BuildNetworkQuery(new GeoBounds{South=43.300,West=5.371,North=43.307,East=5.380},RoadImportLevel.Secondary,true)));
        using var http=new HttpClient{Timeout=TimeSpan.FromSeconds(90)};
        http.DefaultRequestHeaders.UserAgent.ParseAdd("ImportEarthLocation/Overpass-smoke");
        foreach(var (name,query) in queries)
        {
            using var content=new FormUrlEncodedContent(new Dictionary<string,string>{{"data",query}});
            using var response=await http.PostAsync(OverpassProviders.Endpoints[0],content);
            var bytes=await response.Content.ReadAsByteArrayAsync();
            if(name.StartsWith("historical"))
            {
                Check(response.StatusCode==HttpStatusCode.BadRequest,"Old malformed subtraction unexpectedly accepted");
                Console.WriteLine($"CONFIRMED {name}: HTTP 400 (server rejects the old syntax).");
            }
            else
            {
                Check(response.IsSuccessStatusCode,$"{name}: HTTP {(int)response.StatusCode}");
                var data=ImportEarthLocation.Processing.OsmParser.Merge(new[]{bytes},wholeMap:true);
                Check(data.Ways.Values.All(w=>w.NodeIds.All(data.Nodes.ContainsKey)),name+": missing referenced geometry");
                if(name=="network-nonempty-closure")Check(data.Ways.Count>0,"Real network sample unexpectedly empty");
                Console.WriteLine($"PASS {name}: HTTP {(int)response.StatusCode}; bytes={bytes.Length}; nodes={data.Nodes.Count}; ways={data.Ways.Count}; relations={data.Relations.Count}.");
            }
            await Task.Delay(1000);
        }
    }
    sealed class RequestBarrier(int required)
    {
        readonly TaskCompletionSource entered=new(TaskCreationOptions.RunContinuationsAsynchronously);
        readonly TaskCompletionSource released=new(TaskCreationOptions.RunContinuationsAsynchronously);
        int arrivals;
        public Task Entered=>entered.Task;
        public async Task WaitAsync(CancellationToken token)
        {
            var arrival=Interlocked.Increment(ref arrivals);
            if(arrival>required)return;
            if(arrival==required)entered.TrySetResult();
            await released.Task.WaitAsync(token);
        }
        public void Release()=>released.TrySetResult();
    }
    sealed class Handler(Func<int,int,int,HttpResponseMessage> respond,RequestBarrier? barrier=null,bool realTime=false):HttpMessageHandler
    {
        int active; readonly ConcurrentDictionary<int,int> activeTiles=new();
        readonly ConcurrentDictionary<string,int> activeProviders=new();
        public int Peak,PeakPerProvider,Total,Duplicated;
        public ConcurrentDictionary<(int tile,string host),int> Counts=new();
        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request,CancellationToken token)
        {
            var body=Uri.UnescapeDataString((await request.Content!.ReadAsStringAsync(token)).Substring(5).Replace("+"," "));
            var bounds=Regex.Matches(body,@"\(([-0-9.,]+)\)")[0].Groups[1].Value.Split(',');
            var tile=bounds[2]=="0"?-1:int.Parse(bounds[1]);
            var attempt=Counts.AddOrUpdate((tile,request.RequestUri!.Host),1,(_,old)=>old+1);
            var total=Interlocked.Increment(ref Total); var concurrent=Interlocked.Increment(ref active);
            var providerConcurrent=activeProviders.AddOrUpdate(request.RequestUri!.Host,1,(_,old)=>old+1);
            if(activeTiles.AddOrUpdate(tile,1,(_,old)=>old+1)>1) Interlocked.Increment(ref Duplicated);
            lock(this) { Peak=Math.Max(Peak,concurrent); PeakPerProvider=Math.Max(PeakPerProvider,providerConcurrent); }
            try
            {
                if(barrier!=null)await barrier.WaitAsync(token);
                if(realTime)await Task.Delay(1,token);
                else await Task.Yield(); // genuine asynchronous completion, no artificial wall-clock latency
                token.ThrowIfCancellationRequested();
                return respond(tile,attempt,total);
            }
            finally { Interlocked.Decrement(ref active); activeTiles.AddOrUpdate(tile,0,(_,old)=>old-1); activeProviders.AddOrUpdate(request.RequestUri!.Host,0,(_,old)=>old-1); }
        }
    }
}
namespace ImportEarthLocation.Core
{
    public readonly record struct OsmNode(long Id,double Latitude,double Longitude);
    public sealed record OsmWay(long Id,long[] NodeIds,IReadOnlyDictionary<string,string> Tags);
    public readonly record struct OsmMember(string Type,long Id,string Role);
    public sealed record OsmRelation(long Id,IReadOnlyList<(long wayId,bool inner)> Members,IReadOnlyDictionary<string,string> Tags)
    { public IReadOnlyList<OsmMember> OrderedMembers { get; init; }=Array.Empty<OsmMember>(); }
    public sealed class ImportLog { public ConcurrentQueue<string> Lines=new(); public void Info(string x)=>Lines.Enqueue(x); public void Warn(string x)=>Lines.Enqueue(x); public void Diagnostic(string x)=>Lines.Enqueue(x); public void Count(string x,int count=1){} public void Time(string x,double seconds){} }
}
namespace ImportEarthLocation.Geo
{
    public struct GeoBounds { public double South,West,North,East; }
    public static class GeoMath { public static List<GeoBounds> Split4x4(GeoBounds bounds)=>Enumerable.Range(0,16).Select(i=>new GeoBounds{West=i,East=i+1,North=1}).ToList(); }
}
namespace ImportEarthLocation.Data
{
    public static class PublicHttpClient { public static HttpClient Instance=new(); }
}
namespace ImportEarthLocation { public static class Mod { public const string Id="OverpassSmoke"; } }
namespace UnityEngine { public static class Application { public static string persistentDataPath=>Path.GetTempPath(); } }
