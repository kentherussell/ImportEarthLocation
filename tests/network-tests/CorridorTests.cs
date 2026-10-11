using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

internal static class CorridorTests
{
    private static PreparedSegment S(long way,long a,long b,float x1,float z1,float x2,float z2,NetworkKind kind=NetworkKind.Rail)=>
        new(way,a,b,new float3(x1,0,z1),new float3(x2,0,z2),kind,kind==NetworkKind.Rail?"rail":"primary",kind==NetworkKind.Rail?1:2,false,false,false,0)
        {Name="Route",SourceGeometry=new[]{new float3(x1,0,z1),new float3(x2,0,z2)}};
    private static void Check(bool pass,string message){if(!pass)throw new Exception(message);}
    public static void CachedMap(string path,string? snapshot=null,bool cachedTiles=false,double latitude=38.895098,double longitude=-77.036385,RoadImportLevel level=RoadImportLevel.Primary)
    {
        var clock=System.Diagnostics.Stopwatch.StartNew();var log=new ImportLog();
        var files=System.IO.Directory.Exists(path)?System.IO.Directory.GetFiles(path,"*.osm"):new[]{path};
        if(cachedTiles)
        {
            var bounds=ImportEarthLocation.Geo.GeoMath.BoundsFromCenter(latitude,longitude,57344);
            files=ImportEarthLocation.Geo.GeoMath.Split4x4(bounds).Select((tile,index)=>
            {
                var query=ImportEarthLocation.Data.OverpassClient.BuildNetworkQuery(tile,level,true);
                var key=$"overpass-v4:networks:{index}:{query}";
                var hash=Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(key))).ToLowerInvariant();
                var current=System.IO.Path.Combine(path,hash+".osm");
                if(System.IO.File.Exists(current))return current;
                // Keep the existing offline fixture usable after output-only
                // query changes; legacy tiles contain the same full geometry.
                var noWallQuery=System.Text.RegularExpressions.Regex.Replace(query,"way\\[\"barrier\"=\"retaining_wall\"\\]\\([^;]+;","");
                foreach(var candidate in new[]{query,noWallQuery}.SelectMany(q=>new[]{q,q.Replace("out body qt;node(w);out skel qt;","(._;>;);out body;")}))
                {
                    var legacyKey=$"overpass-v4:networks:{index}:{candidate}";
                    var legacyHash=Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(legacyKey))).ToLowerInvariant();
                    var legacyPath=System.IO.Path.Combine(path,legacyHash+".osm");if(System.IO.File.Exists(legacyPath))return legacyPath;
                }
                return current;
            }).ToArray();
            Check(files.All(System.IO.File.Exists),"The existing 16-tile smoke fixture requires complete readable cached network tiles; no download performed");
        }
        var dataset=OsmParser.Merge(files.Select(System.IO.File.ReadAllBytes).ToArray(),log,default,wholeMap:!cachedTiles);
        var request=new ImportRequest(latitude,longitude,new("cached-test","cached-test",57344,false),true){RoadLevel=level,ImportRail=true};
        var raw=NetworkProcessor.BuildSegments(dataset,request,log);
        Console.WriteLine($"Cached fixture: parsed {files.Length} documents; rawSections={raw.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}");
        Check(raw.Count>100,"Cached document did not contain substantial selected-location network geometry");
        var avenues=AvenueConsolidator.Prepare(raw,log,default);
        avenues=RoundaboutPlanner.Prepare(avenues,log,default);
        var rails=RailCorridorConsolidator.Prepare(avenues,log,default);
        Console.WriteLine($"Cached fixture: avenue/rail consolidation complete; sections={rails.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}");
        var normalized=NetworkNormalizer.Normalize(rails,log,default,57344);
        normalized=NetworkHardener.Prepare(normalized,log,default,true);
        float NativeWidth(PreparedSegment s)=>s.Kind==NetworkKind.Rail?12:s.Classification.StartsWith("motorway")||s.Classification.StartsWith("trunk")?Math.Min(5,s.Lanes)*4+5:Math.Max(16,s.Lanes*3.5f+8);
        normalized=NetworkWidthRepair.Prepare(normalized,NativeWidth,57344,log,default);
        foreach(var line in log.Lines.Where(l=>l.StartsWith("PROCESS vanilla-width-complete")))Console.WriteLine(line);
        normalized=normalized.Select(s=>s with{NativeWidth=NativeWidth(s)}).ToArray();
        normalized=NetworkNormalizer.RefineFinalRoadJoins(normalized,log,default,57344);
        var baseline=NetworkNormalizer.Normalize(raw,new ImportLog(),default,57344);
        NetworkNormalizer.Validate(normalized,57344,default);
        var flat=Enumerable.Repeat((ushort)(100f/300*65535),128*128).ToArray();
        var initial=NetworkProcessor.AlignToTerrain(normalized,flat,300,57344,default);
        var profiled=RailVerticalProfiles.Prepare(initial,flat,300,57344,log,default);
        profiled=NetworkVerticalProfiles.Prepare(profiled,flat,300,57344,log,default);
        profiled=OsmRecovery.RepairGeometry(profiled,57344,log,default);
        profiled=NetworkLaneAlignment.Prepare(profiled,log,default,57344);
        profiled=NetworkWidthRepair.Prepare(profiled,s=>s.NativeWidth,57344,log,default);
        profiled=CorridorProfiles.Prepare(profiled,log,default);
        Console.WriteLine(log.Lines.Last(l=>l.StartsWith("PROCESS vanilla-width-complete")));
        Console.WriteLine($"Cached fixture: horizontal placement complete at {clock.Elapsed.TotalSeconds:0.0}s; checking coupled clearances and grades.");
        try { profiled=NetworkGeometrySafety.Repair(profiled,flat,300,57344,log,default,allowQualityWarnings:true); }
        catch { foreach(var line in log.Lines.Where(l=>l.StartsWith("NETWORK SAFETY")))Console.WriteLine(line);throw; }
        Check(profiled.SelectMany(s=>Enumerable.Range(0,33).Select(i=>NetworkNormalizer.Evaluate(s,i/32f))).All(p=>math.all(math.isfinite(p))&&p.y<161),"Cached interchange topology generated nonfinite/sky-high network profiles on flat DEM");
        profiled=NetworkCutPlanner.Prepare(profiled,flat,300,57344,Array.Empty<float3[]>(),Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,default);
        var baselineRaster=flat;var roadbedProfiles=profiled;
        var roadbedRaster=NetworkRoadbed.Prepare(flat,300,57344,profiled,Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,default);
        flat=roadbedRaster;
        profiled=NetworkGeometrySafety.RepairFinalTunnelCover(profiled,flat,300,57344,log,default);
        profiled=CorridorProfiles.Prepare(profiled,log,default,finishOnly:true);
        profiled=NetworkGeometrySafety.RepairFinalTunnelCover(profiled,flat,300,57344,log,default);
        flat=NetworkRoadbed.Refresh(baselineRaster,roadbedRaster,flat,300,57344,roadbedProfiles,profiled,Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,default);
        if(snapshot!=null)System.IO.File.WriteAllText(snapshot,System.Text.Json.JsonSerializer.Serialize(profiled.Select(s=>s with{HorizontalParent=null,SourceGeometry=Array.Empty<float3>()}).ToArray(),SnapshotOptions));
        try{NetworkContinuity.Validate(normalized,profiled,log,default,allowQualityWarnings:true);}catch{foreach(var line in log.Lines.Where(l=>l.Contains("Rail overlap remains")||l.StartsWith("PROCESS continuity")))Console.WriteLine(line);throw;}
        NetworkGeometrySafety.ValidateEnvelope(profiled,57344,300,default);
        NetworkGeometrySafety.ValidateTunnelCover(profiled,flat,300,57344,default);
        NetworkGeometrySafety.Validate(profiled,log,default,allowQualityWarnings:true);
        foreach(var line in log.Lines.Where(l=>l.StartsWith("NETWORK SAFETY FINAL")))Console.WriteLine(line);
        foreach(var line in log.Lines.Where(l=>l.StartsWith("PROCESS corridor-profiles")||l.StartsWith("NETWORK QUALITY class=")||l.StartsWith("NETWORK QUALITY JOINS")))Console.WriteLine(line);
        foreach(var line in log.Lines.Where(l=>l.StartsWith("ROAD FINAL AUDIT")||l.StartsWith("PROCESS connected-road-grades complete")))Console.WriteLine(line);
        Check(normalized.Where(s=>s.Kind==NetworkKind.Rail).All(s=>s.Lanes>=1&&s.Lanes<=2),"Real-map native rail count is outside the single/double supported policy");
        Check(normalized.SelectMany(s=>new[]{(key:(s.Kind,s.Layer,s.Bridge,s.Tunnel,s.StartNodeId),p:s.Start),(key:(s.Kind,s.Layer,s.Bridge,s.Tunnel,s.EndNodeId),p:s.End)}).GroupBy(p=>p.key).All(g=>g.All(p=>math.distance(p.p,g.First().p)<.01f)),"Real-map consolidation produced conflicting shared-node positions");
        foreach(var line in log.Lines.Where(l=>l.StartsWith("RAIL TOTAL")||l.StartsWith("AVENUE TOTAL")||l.StartsWith("PROCESS normalization complete")))Console.WriteLine(line);
        Console.WriteLine($"PASS: cached location offline; latitude={latitude}; longitude={longitude}; rawRoadSpans={raw.Count(s=>s.Kind==NetworkKind.Road)}; rawRailSpans={raw.Count(s=>s.Kind==NetworkKind.Rail)}; consolidatedRailSpans={rails.Count(s=>s.Kind==NetworkKind.Rail)}; fittedRailBefore={baseline.Count(s=>s.Kind==NetworkKind.Rail)}; fittedRailAfter={normalized.Count(s=>s.Kind==NetworkKind.Rail)}; outputSections={normalized.Count}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}; flatDemFixture=true; noDownloadsOrGame=true; nativeRenderingNotTested=true");
    }
    private static readonly System.Text.Json.JsonSerializerOptions SnapshotOptions=new(){IncludeFields=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals,Converters={new Float3Converter()}};
    private sealed class Float3Converter:System.Text.Json.Serialization.JsonConverter<float3>
    {
        public override float3 Read(ref System.Text.Json.Utf8JsonReader reader,Type type,System.Text.Json.JsonSerializerOptions options)
        {
            if(reader.TokenType==System.Text.Json.JsonTokenType.StartArray)
            {reader.Read();var x=reader.GetSingle();reader.Read();var y=reader.GetSingle();reader.Read();var z=reader.GetSingle();reader.Read();return new(x,y,z);}
            if(reader.TokenType!=System.Text.Json.JsonTokenType.StartObject)throw new System.Text.Json.JsonException("Expected diagnostic vector.");
            var point=new float3();var components=0;
            while(reader.Read()&&reader.TokenType!=System.Text.Json.JsonTokenType.EndObject)
            {
                var key=reader.GetString();reader.Read();var value=reader.GetSingle();
                if(key=="x")point.x=value;else if(key=="y")point.y=value;else if(key=="z")point.z=value;else throw new System.Text.Json.JsonException("Unexpected vector member.");
                components++;
            }
            if(components!=3)throw new System.Text.Json.JsonException("Incomplete diagnostic vector.");return point;
        }
        public override void Write(System.Text.Json.Utf8JsonWriter writer,float3 value,System.Text.Json.JsonSerializerOptions options)
        {writer.WriteStartArray();writer.WriteNumberValue(value.x);writer.WriteNumberValue(value.y);writer.WriteNumberValue(value.z);writer.WriteEndArray();}
    }
    public static void AuditDiagnostic(string directory,string stage)
    {
        if(stage!="before-cover"&&stage!="native-before-cover"&&stage!="native-plan")throw new ArgumentException("Use a recorded geometry stage.");
        using var metadata=System.Text.Json.JsonDocument.Parse(File.ReadAllText(Path.Combine(directory,stage+"-geometry.json")));
        var root=metadata.RootElement;
        if(root.GetProperty("network").GetProperty("Truncated").GetBoolean())throw new InvalidOperationException("Network artifact is truncated; do not treat a partial replay as full coverage.");
        var options=new System.Text.Json.JsonSerializerOptions(SnapshotOptions);
        options.Converters.Add(new System.Text.Json.Serialization.JsonStringEnumConverter());
        var sections=new List<PreparedSegment>();
        foreach(var line in File.ReadLines(Path.Combine(directory,stage+"-network.ndjson")))
        {
            using var record=System.Text.Json.JsonDocument.Parse(line);
            if(record.RootElement.GetProperty("SourcePointsTruncated").GetBoolean())throw new InvalidOperationException("Source geometry is truncated; only a limited regional audit is possible.");
            sections.Add(System.Text.Json.JsonSerializer.Deserialize<PreparedSegment>(line,options)!);
        }
        if(sections.Count!=root.GetProperty("sectionsRequested").GetInt32())throw new InvalidOperationException("Recorded section count differs from geometry metadata.");
        var bytes=File.ReadAllBytes(Path.Combine(directory,stage+"-heightmap.u16"));
        if(bytes.Length!=root.GetProperty("heightSamples").GetInt32()*2)throw new InvalidOperationException("Recorded terrain size differs from metadata.");
        var heights=new ushort[bytes.Length/2];for(var i=0;i<heights.Length;i++)heights[i]=(ushort)(bytes[i*2]|bytes[i*2+1]<<8);
        var scale=root.GetProperty("heightScaleMeters").GetSingle();var width=root.GetProperty("mapWidthMeters").GetDouble();var log=new ImportLog();
        NetworkGeometrySafety.ValidateEnvelope(sections,width,scale,default);
        NetworkGeometrySafety.Validate(sections,log,default,allowQualityWarnings:true);
        try { NetworkGeometrySafety.ValidateTunnelCover(sections,heights,scale,width,default);Console.WriteLine("Diagnostic actual-raster roof audit passed."); }
        finally { foreach(var line in log.Lines)Console.WriteLine(line); }
        Console.WriteLine($"Diagnostic replay: stage={stage}, sections={sections.Count}, terrain={heights.Length}; original raster, no download, no native game mutation. This is not rendered acceptance.");
    }
    private static void DiagnosticRoundtrip()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-diagnostic-replay-"+Guid.NewGuid().ToString("N"));
        try
        {
            var archive=new RunArchive(root,new { latitude=0,longitude=0 },typeof(CorridorTests).Assembly.Location);
            var tunnel=S(77,701,702,-100,0,100,0) with { Tunnel=true,Start=new float3(-100,15,0),End=new float3(100,15,0),NativeWidth=4,SourceGeometry=new[]{new float3(-100,15,0),new float3(100,15,0)} };
            var heights=Enumerable.Repeat((ushort)(50f/100*65535),16*16).ToArray();
            DiagnosticArtifacts.CaptureGeometry(archive,"native-plan",new[]{tunnel},heights,100,1000);
            AuditDiagnostic(archive.DirectoryPath,"native-plan");
            DiagnosticArtifacts.CaptureGeometry(archive,"before-cover",new[]{tunnel},new ushort[16*16],100,1000);
            var rejected=false;try{AuditDiagnostic(archive.DirectoryPath,"before-cover");}catch(InvalidOperationException e){rejected=e.Message.Contains("terrain exposes");}
            Check(rejected,"Diagnostic replay substituted flat terrain or missed an exposed roof in the original raster");
            Console.WriteLine("PASS: diagnostic original-raster replay preserves controls/offsets and distinguishes covered from exposed tunnels without a download or game mutation.");
        }
        finally{if(Directory.Exists(root))Directory.Delete(root,true);}
    }
    public static void AuditSnapshot(string path,bool repair=false)
    {
        var sections=System.Text.Json.JsonSerializer.Deserialize<PreparedSegment[]>(System.IO.File.ReadAllText(path),SnapshotOptions)!;
        var log=new ImportLog();var flat=Enumerable.Repeat((ushort)(100f/300*65535),128*128).ToArray();
        try
        {
            if(repair)
            {
                sections=NetworkWidthRepair.Prepare(sections,s=>s.NativeWidth,57344,log,default).ToArray();
                sections=NetworkGeometrySafety.Repair(sections,flat,300,57344,log,default,allowQualityWarnings:true).ToArray();
                flat=NetworkRoadbed.Prepare(flat,300,57344,sections,Array.Empty<float3[]>(),Array.Empty<float3[]>(),log,default);
                sections=NetworkGeometrySafety.RepairFinalTunnelCover(sections,flat,300,57344,log,default).ToArray();
                // Exercise the finishing repair in this existing offline smoke,
                // independently of whether this snapshot's roadbed exposes a roof.
                var coverProbe=sections.Where(s=>s.Tunnel).ToArray();
                if(coverProbe.Length>0)
                {
                    var lowered=flat.Select(y=>(ushort)Math.Max(0,y-5f/300*65535)).ToArray();
                    var fixedCover=NetworkGeometrySafety.RepairFinalTunnelCover(coverProbe,lowered,300,57344,log,default);
                    NetworkContinuity.Validate(coverProbe,fixedCover,log,default,allowQualityWarnings:true);
                    Check(fixedCover.Count==coverProbe.Length&&fixedCover.Where((s,i)=>math.any(s.Start.xz!=coverProbe[i].Start.xz)||math.any(s.End.xz!=coverProbe[i].End.xz)).Count()==0,"Final tunnel cover repair deleted sections or shifted horizontal alignment");
                }
            }
            NetworkGeometrySafety.Validate(sections,log,default,allowQualityWarnings:repair);
        }
        catch{foreach(var line in log.Lines.Where(l=>l.StartsWith("NETWORK SAFETY")))Console.WriteLine(line);throw;}
        try{NetworkContinuity.Validate(sections,sections,log,default,allowQualityWarnings:repair);}catch{foreach(var line in log.Lines.Where(l=>l.Contains("Rail overlap remains")||l.StartsWith("PROCESS continuity")))Console.WriteLine(line);throw;}
        NetworkGeometrySafety.ValidateEnvelope(sections,57344,300,default);
        NetworkGeometrySafety.ValidateTunnelCover(sections,flat,300,57344,default);
        foreach(var line in log.Lines.Where(l=>l.StartsWith("NETWORK SAFETY FINAL")||l.StartsWith("PROCESS continuity")||l.StartsWith("PROCESS vanilla-width-complete")||l.StartsWith("NETWORK SAFETY repaired")||l.StartsWith("PROCESS final-tunnel-cover")))Console.WriteLine(line);
    }
    public static void Run()
    {
        DiagnosticRoundtrip();
        RoadDesignChecks();
        RailSwitchChecks();
        var triangle=new[]{S(901,9001,9003,0,0,80,0),S(902,9001,9002,0,0,55,1),S(903,9002,9003,55,1,80,0),S(903,9003,9004,80,0,200,0)};
        var triangleResult=RailCorridorConsolidator.Prepare(triangle,new ImportLog(),default);
        NetworkContinuity.Validate(triangleResult,triangleResult,new ImportLog(),default,allowQualityWarnings:true);
        Check(triangleResult.Any(s=>s.EndNodeId==9004),"Consolidating a short redundant switch triangle deleted its unique continuation");
        var log=new ImportLog();var main=S(1,1,2,0,0,300,0);
        var parallel=new[]{S(2,3,4,0,6,150,6),S(2,4,5,150,6,300,6)};
        var branch=S(3,4,6,150,6,150,100);
        var source=new[]{main,parallel[0],parallel[1],branch};
        var result=RailCorridorConsolidator.Prepare(source,log,default);
        Check(result.Count==3&&result.Count(s=>s.WayId==1)==2,"Parallel rails were not folded into a single split axis");
        var transferred=result.Single(s=>s.WayId==3);
        Check(transferred.Start.z==0&&result.Count(s=>s.StartNodeId==transferred.StartNodeId||s.EndNodeId==transferred.StartNodeId)==3,"Rail branch lost its native junction");
        var reverse=RailCorridorConsolidator.Prepare(new[]{main,S(2,5,3,300,6,0,6)},log,default);
        Check(reverse.Count==1,"Reverse-oriented parallel rail retained");
        var dividedAxis=new[]{S(1,1,2,0,0,100,0),S(1,2,7,100,0,200,0),S(1,7,6,200,0,300,0),S(2,3,5,300,6,0,6)};
        Check(RailCorridorConsolidator.Prepare(dividedAxis,log,default).Count==3,"Different raw vertex spacing prevented rail consolidation");
        var partial=RailCorridorConsolidator.Prepare(new[]{main,S(2,3,4,0,6,200,6) with{RailService="siding"},S(2,4,5,200,6,400,6) with{RailService="siding"}},log,default);
        Check(partial.Count(s=>s.WayId==2)==2&&partial.Any(s=>s.WayId==2&&s.End.x==400),"Independent siding or its unique tail was deleted");
        var dense=new List<PreparedSegment>{main};for(var i=0;i<8;i++)dense.Add(S(10+i,100+i*2,101+i*2,0,2+i*2,300,2+i*2) with{RailService="yard"});
        Check(RailCorridorConsolidator.Prepare(dense,log,default).Count==dense.Count,"Independent yard tracks were destructively consolidated");
        Check(RailCorridorConsolidator.Prepare(new[]{main,S(2,3,4,0,6,300,6) with{Layer=1}},log,default).Count==2,"Different rail levels were destructively consolidated");
        foreach(var guarded in new[]{S(2,3,4,0,30,300,30),S(2,3,4,0,6,300,6) with{Bridge=true},S(2,3,4,0,6,300,6) with{Tunnel=true},S(2,3,4,150,-100,150,100)})
            Check(RailCorridorConsolidator.Prepare(new[]{main,guarded},log,default).Count==2,"Rail crossing/level/structure guard failed");
        var bridge=S(3,5,6,300,6,400,6) with{Bridge=true,Layer=1};
        var protectedResult=RailCorridorConsolidator.Prepare(new[]{main,parallel[0],parallel[1],bridge},log,default);
        var portal=protectedResult.Single(s=>s.WayId==3);
        Check(portal.Start.z==0&&protectedResult.Any(s=>s.WayId==1&&s.EndNodeId==portal.StartNodeId),"Rail structural transition disconnected");
        foreach(var tunnel in new[]{false,true})
        {
            var structural=RailCorridorConsolidator.Prepare(new[]{main with{Bridge=!tunnel,Tunnel=tunnel},S(2,3,4,0,6,300,6) with{Bridge=!tunnel,Tunnel=tunnel}},log,default);
            Check(structural.Count==1&&structural[0].Lanes==2,"Parallel structural rail was not consolidated");
        }
        Check(RailCorridorConsolidator.Prepare(source,log,default).Select(s=>(s.WayId,s.StartNodeId,s.EndNodeId,s.Start,s.End)).SequenceEqual(result.Select(s=>(s.WayId,s.StartNodeId,s.EndNodeId,s.Start,s.End))),"Rail consolidation is not deterministic");
        NetworkNormalizer.Validate(NetworkNormalizer.Normalize(result,log,default,1000),1000,default);
        using(var cancellation=new CancellationTokenSource()){cancellation.Cancel();try{RailCorridorConsolidator.Prepare(source,log,cancellation.Token);throw new Exception("Rail cancellation ignored");}catch(OperationCanceledException){}}
        var avenue=new[]{S(50,-100,-101,0,0,25,.4f,NetworkKind.Road),S(50,-101,-102,25,.4f,50,0,NetworkKind.Road),S(50,-102,-103,50,0,75,.4f,NetworkKind.Road)};
        Check(NetworkNormalizer.Normalize(avenue,log,default,1000).Count<avenue.Length,"Synthetic avenue nodes still block smoothing");
        var sharp=new[]{S(51,-110,-111,0,0,30,0,NetworkKind.Road),S(51,-111,-112,30,0,30,30,NetworkKind.Road)};
        Check(NetworkNormalizer.Normalize(sharp,log,default,1000).Count==2,"Intentional corner rounded across topology");
        var divided=new[]{S(80,300,301,0,0,100,0,NetworkKind.Road) with{OneWay=true},S(80,301,302,100,0,200,0,NetworkKind.Road) with{OneWay=true},S(80,302,303,200,0,300,0,NetworkKind.Road) with{OneWay=true},S(81,304,305,300,10,200,10,NetworkKind.Road) with{OneWay=true},S(81,305,306,200,10,100,10,NetworkKind.Road) with{OneWay=true},S(81,306,307,100,10,0,10,NetworkKind.Road) with{OneWay=true},S(82,299,300,-100,0,0,0,NetworkKind.Road) with{Bridge=true},S(83,307,308,0,10,-100,10,NetworkKind.Road) with{Bridge=true}};
        var dividedResult=AvenueConsolidator.Prepare(divided,log,default);
        Check(dividedResult.Any(s=>s.ConsolidatedAvenue)&&dividedResult.Single(s=>s.WayId==82).EndNodeId==300&&dividedResult.Single(s=>s.WayId==83).StartNodeId==307,"Structural approach preservation unnecessarily blocked the entire interior avenue");
        var terrain=new ushort[128*128];for(var z=0;z<128;z++)for(var x=0;x<128;x++){var wx=(x/127f-.5f)*1000;terrain[z*128+x]=(ushort)Math.Round((50+3*Math.Cos(wx/25))/100*65535);}
        var roads=new List<PreparedSegment>();for(var i=0;i<12;i++)roads.Add(S(70,200+i,201+i,-150+i*25,0,-125+i*25,0,NetworkKind.Road));
        var graded=NetworkVerticalProfiles.Prepare(roads,terrain,100,1000,log,default);
        Check(graded.Count>=roads.Count&&graded.All(s=>NetworkVerticalProfiles.WithinGradeCeiling(s,NetworkVerticalProfiles.GradeLimit(s))),"Smoothing dropped repairable road spans or exceeded grade ceiling");
        var interior=graded.SelectMany(s=>new[]{(id:s.StartNodeId,y:s.Start.y),(id:s.EndNodeId,y:s.End.y)}).Where(p=>p.id>200&&p.id<212);
        Check(interior.Max(p=>p.y)-interior.Min(p=>p.y)<3,"Longitudinal profile still follows every terrain hump");
        Console.WriteLine($"ROAD SMOOTHNESS FIXTURE inputReliefMeters=6; interiorNodeReliefMeters={interior.Max(p=>p.y)-interior.Min(p=>p.y):0.###}; maximumGradePercent={graded.Max(NetworkVerticalProfiles.MaximumGrade)*100:0.###}; inputSections={roads.Count}; outputSections={graded.Count}; offlineSynthetic=true");
        Check(graded.SelectMany(s=>new[]{(id:s.StartNodeId,y:s.Start.y),(id:s.EndNodeId,y:s.End.y)}).GroupBy(p=>p.id).All(g=>g.Max(p=>p.y)-g.Min(p=>p.y)<.001),"Vertical smoothing broke shared endpoint heights");
        Check(log.Lines.Any(l=>l.Contains("RAIL TOTAL"))&&log.Lines.Any(l=>l.Contains("longitudinal-road-smoothing")),"Corridor diagnostics missing");
        IndexedGeometryChecks();
        Console.WriteLine("PASS: bounded two-track mainline pairing, independent yards/sidings/levels, reversed tracks, transferred branches, unique tails, source-evidenced switches, determinism, cancellation, synthetic-node smoothing, sharp corners and rolling longitudinal road grades.");
    }
    private static void RailSwitchChecks()
    {
        var axis=S(901,901,902,-100,0,100,0);
        var spur=S(902,903,904,0,60,0,5);
        var continuation=S(903,904,905,0,5,60,45);
        var unchanged=RailJunctionRepair.Prepare(new[]{axis,spur,continuation},new ImportLog(),default);
        Check(unchanged.Count==3&&unchanged[1].EndNodeId==904&&unchanged[1].End.Equals(spur.End),"Nearby rails were joined without source-node switch evidence");
        var incoming=S(901,901,904,-100,0,0,0);
        var outgoing=S(904,904,906,0,0,100,0);
        var connected=RailJunctionRepair.Prepare(new[]{incoming,outgoing,spur,continuation},new ImportLog(),default);
        Check(connected.Count==4&&connected.Count(s=>s.StartNodeId==904||s.EndNodeId==904)==4,"Mapped switch lost an incident edge");
        Check(connected.Where(s=>s.StartNodeId==904).All(s=>s.Start.Equals(float3.zero))&&connected.Where(s=>s.EndNodeId==904).All(s=>s.End.Equals(float3.zero)),"Mapped switch was not canonicalized to one native anchor");
        Check(spur.End.z==5&&axis.Start.x==-100,"Rail repair mutated its input");
    }
    private static void RoadDesignChecks()
    {
        var log=new ImportLog();
        PreparedSegment Road(long way,long from,long to,float x1,float z1,float x2,float z2,int lanes=3)=>S(way,from,to,x1,z1,x2,z2,NetworkKind.Road) with{Classification="motorway",OneWay=true,Lanes=lanes,Reference="TEST"};
        var laneRun=new[]{Road(701,1,2,-300,0,-50,0),Road(702,2,3,-50,0,50,0,2),Road(703,3,4,50,0,300,0)};
        var normalized=NetworkNormalizer.Normalize(laneRun,log,default,1000);
        Check(normalized.All(s=>s.Lanes==3)&&normalized.First().StartNodeId==1&&normalized.Last().EndNodeId==4,"Short 3-2-3 mainline lane blip not removed or unique terminal topology changed");
        var junction=Road(704,2,5,-50,0,-50,100,1) with{Classification="motorway_link"};
        Check(NetworkNormalizer.Normalize(laneRun.Concat(new[]{junction}).ToArray(),log,default,1000).Single(s=>s.WayId==702).Lanes==2,"Meaningful merge lane change was erased");
        var merge=new[]{Road(720,30,31,-300,0,0,0),Road(721,31,32,0,0,250,0),Road(722,32,33,250,0,450,0),Road(723,34,31,-120,-80,0,0,1) with{Classification="motorway_link"}};
        var merging=NetworkNormalizer.Normalize(merge,log,default,1000);
        Check(merging.Where(s=>s.Classification=="motorway").All(s=>s.Lanes==3)&&merging.Any(s=>s.StartNodeId==31),"Synthetic entrance widening introduced a lane-count blip or erased the merge junction");
        var avenueRun=new[]{Road(730,50,51,-400,200,-50,200,6),Road(731,51,52,-50,200,0,200,4),Road(732,52,53,0,200,50,200,4),Road(733,53,54,50,200,400,200,6)}
            .Select(s=>s with{Classification="primary",OneWay=false}).ToArray();
        Check(NetworkNormalizer.Normalize(avenueRun,log,default,1000).All(s=>s.Lanes==6),"Multi-segment ordinary-road width fluctuation was not normalized");
        var raster=Enumerable.Repeat((ushort)(40f/100*65535),128*128).ToArray();
        var upper=Road(710,10,11,-100,0,100,0) with{Bridge=true,Layer=1};
        var crossing=new[]{upper,Road(711,9,10,-400,0,-100,0),Road(712,11,12,100,0,400,0),Road(713,20,21,0,-400,0,400)};
        var solved=NetworkVerticalProfiles.Prepare(crossing,raster,100,1000,log,default);
        var deck=solved.Single(s=>s.WayId==710);var lower=solved.Single(s=>s.WayId==713);
        var separation=NetworkNormalizer.Evaluate(deck,.5f).y-NetworkNormalizer.Evaluate(lower,.5f).y;
        Check(separation>=7,"Bridge deck allowance leaves less than 5 m clear space");
        Check(solved.Count==crossing.Length&&solved.Count(s=>s.Bridge)==1,"Clearance fix split/deleted roads or created extra bridges");
        Check(solved.All(s=>NetworkVerticalProfiles.MaximumGrade(s)<=.08f),"Whole-span clearance created excessive approach slopes");
        Check(NetworkVerticalProfiles.MinimumBridgeClearance==5,"Minimum clearance changed without authorization");
        var sharedGroup=new[]{Road(740,40,41,-100,0,0,0) with{Bridge=true,Layer=1},Road(741,41,42,0,0,100,0) with{Bridge=true,Layer=1},
            Road(742,40,43,-100,0,100,100) with{Control1=new float3(-40,0,-150),Control2=new float3(60,0,-100)}};
        var bounded=NetworkVerticalProfiles.Prepare(sharedGroup,raster,100,1000,new ImportLog(),default);
        Check(bounded.SelectMany(s=>Enumerable.Range(0,33).Select(i=>NetworkNormalizer.Evaluate(s,i/32f))).All(p=>math.isfinite(p.y)&&p.y<40.1f),"Connected lower approach triggered self-clearance runaway");
        Check(bounded.Count==3,"Self-clearance fallback deleted or split connected roads");
        var repeated=Enumerable.Range(0,25).Select(i=>Road(800+i,100+i*2,101+i*2,-300,i-12,300,i-12) with{Bridge=true,Layer=1}).Concat(new[]{Road(900,300,301,0,-400,0,400)}).ToArray();
        var boundedCrossings=NetworkVerticalProfiles.Prepare(repeated,raster,100,1000,new ImportLog(),default);
        Check(boundedCrossings.All(s=>s.Start.y<80&&s.End.y<80&&NetworkVerticalProfiles.MaximumGrade(s)<.08f),"Repeated crossing constraints caused sky roads");
        foreach(var side in new[]{-1,1})
        {
            var mainIn=Road(950,400,401,-300,0,0,0) with{NativeLaneOffsets=new[]{-3.5f,0,3.5f}};
            var mainOut=Road(951,401,402,0,0,300,0) with{NativeLaneOffsets=new[]{-3.5f,0,3.5f}};
            var ramp=Road(952,403,401,-200,side*80,0,0,1) with{Classification="motorway_link",Control1=new float3(-150,0,side*60),Control2=new float3(-40,0,side*8),NativeLaneOffsets=new[]{0f}};
            var aligned=NetworkLaneAlignment.Prepare(new[]{mainIn,mainOut,ramp},new ImportLog(),default,1000);
            var refined=NetworkNormalizer.RefineFinalRoadJoins(aligned,new ImportLog(),default,1000).Single(s=>s.WayId==952);
            Check(math.all(refined.End==ramp.End)&&refined.EndNodeId==ramp.EndNodeId,"Lane alignment detached the shared junction");
            Check(Math.Sign(refined.EndCurveOffset.z)==side&&Math.Abs(Math.Abs(refined.EndCurveOffset.z)-3.5f)<.001&&Math.Abs(refined.CourseControl2.z)<3.6f,"Ramp course did not select the closest outside lane exactly once");
        }
        Console.WriteLine($"ROAD DESIGN FIXTURE clearanceDeckSeparationMeters={separation:0.###}; inputSections={crossing.Length}; outputSections={solved.Count}; shortLaneBlipsNormalized=true; mergeLaneChangesPreserved=true; offlineSynthetic=true");
    }
    private static void IndexedGeometryChecks()
    {
        bool Reference(PreparedSegment s)
        {
            var tolerance=NetworkNormalizer.Tolerance(s.Kind);var a=s.Start;var b=s.Control1!.Value;var c=s.Control2!.Value;var d=s.End;
            var bound=math.distance(a,b)+math.distance(b,c)+math.distance(c,d);var count=Math.Min(2048,Math.Max(32,(int)Math.Ceiling(6*bound/tolerance)));
            var samples=Enumerable.Range(0,count+1).Select(i=>NetworkNormalizer.Evaluate(s,i/(float)count)).ToArray();var allowance=tolerance-3*bound/(2*count);
            float Distance(float3 p,IReadOnlyList<float3> line){var result=float.MaxValue;for(var i=1;i<line.Count;i++)result=Math.Min(result,NetworkNormalizer.DistanceToSegment(p,line[i-1],line[i]));return result;}
            if(allowance<=0||samples.Any(p=>Distance(p,s.SourceGeometry)>allowance))return false;
            for(var i=1;i<s.SourceGeometry.Count;i++){var first=s.SourceGeometry[i-1];var last=s.SourceGeometry[i];var steps=Math.Max(1,(int)Math.Ceiling(math.distance(first,last)/(tolerance/2)));for(var j=0;j<=steps;j++)if(Distance(math.lerp(first,last,j/(float)steps),samples)>tolerance*.75f)return false;}
            return true;
        }
        foreach(var kind in new[]{NetworkKind.Road,NetworkKind.Rail})foreach(var shift in new[]{0f,.3f,2f,8f})
        {
            var curve=S(500,1,2,0,0,150,35,kind) with{Control1=new float3(50,0,0),Control2=new float3(100,0,35)};
            var points=Enumerable.Range(0,65).Select(i=>NetworkNormalizer.Evaluate(curve,i/64f)).ToArray();
            var candidate=curve with{SourceGeometry=points,Control1=curve.Control1+new float3(0,0,shift),Control2=curve.Control2+new float3(0,0,shift)};
            Check(NetworkNormalizer.WithinTolerance(candidate)==Reference(candidate),"Indexed nearest-segment validation disagrees with the original exhaustive test");
        }
    }
}
