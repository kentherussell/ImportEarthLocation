using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Game;
using Game.Common;
using Game.Objects;
using Game.Prefabs;
using Game.Prefabs.Climate;
using Game.Rendering;
using Game.Routes;
using Game.SceneFlow;
using Game.Simulation;
using Game.UI;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Transform = Game.Objects.Transform;
using WaterSourceData = Game.Simulation.WaterSourceData;

namespace ImportEarthLocation.Integration;

/// <summary>Read-only native snapshots. Diagnostic sidecars never restore or modify a map.</summary>
public static class NativeDiagnostics
{
    public static bool SurfaceApplied, AirApplied, ClimateApplied, WaterApplied;
    private static string _run = "";
    private static string? _phase;
    private static long _map, _generation;
    private static DateTime _due;
    private static ImportLog? _log;
    private static JArray _expectedAir = new();
    private static JArray _expectedWater = new();
    private static readonly System.Collections.Generic.List<float3> WaterProbes = new();
    private static readonly System.Collections.Generic.List<float3> WaterSourcePositions = new();
    private static readonly System.Collections.Generic.List<RiverSourcePlan> WaterPlans = new();
    private static readonly System.Collections.Generic.List<RiverSourcePlan> PlannedWater = new();
    private static readonly System.Collections.Generic.Dictionary<long,(int wet,int total,int flooded,float level)> WaterObservations = new();
    private static DateTime _waterStarted, _waterDue;
    private static long _waterMap, _waterGeneration;
    private static int _waterChecks;
    private static ImportLog? _waterLog;
    private static string? _expectedClimate;
    private static string? _expectedLocalSurface, _expectedWorldSurface;
    private static int _airPendingInitialization;
    private static string Root => Path.Combine(Application.persistentDataPath,"ModsData",Mod.Id,"diagnostics");

    public static void Begin(PreparedImport prepared, ImportLog log)
    {
        SurfaceApplied=AirApplied=ClimateApplied=WaterApplied=false;
        _expectedWater=new JArray(); WaterProbes.Clear();WaterSourcePositions.Clear();WaterPlans.Clear();PlannedWater.Clear();WaterObservations.Clear();
        if(prepared.Water!=null)PlannedWater.AddRange(prepared.Water.Sources);
        foreach(var plan in PlannedWater)WaterProbes.AddRange(plan.ChannelProbes.Count>0?plan.ChannelProbes:new[]{plan.DownstreamProbe});
        _waterLog=null; _waterChecks=0;
        _expectedAir=new JArray();_expectedClimate=_expectedLocalSurface=_expectedWorldSurface=null;
        _run=Guid.NewGuid().ToString("N");
        log.Diagnostic($"RUN id={_run}; modBuildMvid={typeof(NativeDiagnostics).Assembly.ManifestModule.ModuleVersionId}; gameAssemblyVersion={typeof(TerrainSystem).Assembly.GetName().Version}; mapSession={prepared.Request.MapSessionId}; generation={prepared.Request.Generation}; latitude={prepared.Request.Latitude:R}; longitude={prepared.Request.Longitude:R}; mapWidth={prepared.Request.MapSize.WidthMeters:R}; surfacesRequested={prepared.Request.ApplyGroundSurfaces}; airRequested={prepared.Request.CreateAirConnections}");
    }

    public static void RememberAir(string name,string prefab,float3 position)
    { _expectedAir.Add(AirRecord(name,prefab,position,true)); }
    public static void RememberClimate(string name) => _expectedClimate=name;
    public static void RememberWater(float3 position, WaterSourceData source, RiverSourcePlan plan)
    { _expectedWater.Add(WaterRecord(position,source)); WaterSourcePositions.Add(position);WaterPlans.Add(plan); }
    public static void RememberSurface(string label,string hash)
    { if(label=="local")_expectedLocalSurface=hash;else _expectedWorldSurface=hash; }

    public static void Queue(string phase, ImportLog log)
    {
        if(phase=="after-load"){_run="";_waterLog=null;NativeNetworkDiagnostics.Clear();}
        if(phase=="after-apply"&&WaterProbes.Count>0)
        { _waterStarted=DateTime.UtcNow; _waterDue=_waterStarted.AddSeconds(60); _waterMap=ImportSession.Map; _waterGeneration=ImportSession.Generation; _waterChecks=0; _waterLog=log; }
        _phase=phase; _map=ImportSession.Map; _generation=ImportSession.Generation;
        _due=DateTime.UtcNow.AddSeconds(5); _log=log;
        log.Diagnostic($"SNAPSHOT QUEUED phase={phase}; mapSession={_map}; delaySeconds=5; readOnly=true");
    }

    public static void Poll(World world, EntityManager manager)
    {
        if(_waterLog!=null)
        {
            if(_waterMap!=ImportSession.Map||_waterGeneration!=ImportSession.Generation)_waterLog=null;
            else if(DateTime.UtcNow>=_waterDue&&ImportSession.Ready&&!GameManager.instance.isGameLoading&&GameManager.instance.gameMode==GameMode.Editor)
            {
                WaterProbeState(world,"flow-check",_waterLog,(DateTime.UtcNow-_waterStarted).TotalSeconds);
                if(++_waterChecks>=2)_waterLog=null; else _waterDue=_waterStarted.AddSeconds(180);
            }
        }
        if(_phase==null)return;
        if(_map!=ImportSession.Map || _phase=="after-apply"&&_generation!=ImportSession.Generation){_log?.Diagnostic($"SNAPSHOT DISCARDED phase={_phase}; oldMapSession={_map}; activeMapSession={ImportSession.Map}; reason=stale-map-or-generation");_phase=null;_log=null;return;}
        if(DateTime.UtcNow<_due||!ImportSession.Ready||GameManager.instance.isGameLoading||GameManager.instance.gameMode!=GameMode.Editor)return;
        var phase=_phase;var log=_log!;_phase=null;_log=null;
        try { Snapshot(world,manager,phase,log); }
        catch(Exception exception){log.Diagnostic($"SNAPSHOT UNAVAILABLE phase={phase}: {exception}");}
    }

    public static string PixelHash(byte[] encodedPixels) => Hash(encodedPixels);
    private static string Hash(byte[] bytes)
    { using var hash=SHA256.Create();return BitConverter.ToString(hash.ComputeHash(bytes)).Replace("-","").ToLowerInvariant(); }

    private static void Snapshot(World world,EntityManager manager,string phase,ImportLog log)
    {
        var terrain=world.GetOrCreateSystemManaged<TerrainSystem>();
        var data=terrain.GetHeightData(false);
        if(!data.heights.IsCreated||data.heights.Length==0||data.heights.Length>16_777_216)throw new InvalidOperationException("Native height buffer unavailable/too large for bounded diagnostic fingerprint.");
        var values=data.heights.ToArray();var bytes=new byte[values.Length*2];Buffer.BlockCopy(values,0,bytes,0,bytes.Length);
        var identity=new JObject { ["heightHash"]=Hash(bytes),["resolution"]=data.resolution.ToString(),["area"]=terrain.playableArea.ToString(),["scaleOffset"]=terrain.heightScaleOffset.ToString() };
        var key=Hash(System.Text.Encoding.UTF8.GetBytes(identity.ToString(Newtonsoft.Json.Formatting.None)));
        var current=new JObject { ["version"]=1,["run"]=_run,["capturedUtc"]=DateTime.UtcNow.ToString("O"),["terrain"]=identity };
        var prefabs=world.GetOrCreateSystemManaged<PrefabSystem>();
        var materials=world.GetOrCreateSystemManaged<TerrainMaterialSystem>();
        Try("surface",()=>new JObject { ["local"]=TextureSignature(NativeSurfaceBridge.NativeTexture(materials,false)),["world"]=TextureSignature(NativeSurfaceBridge.NativeTexture(materials,true)) });
        Try("air",()=>AirState(manager,prefabs,world.GetOrCreateSystemManaged<NameSystem>(),phase,log));
        if(phase=="after-load"&&_airPendingInitialization>0){current.Remove("air");log.Diagnostic("AIR load readback initialization still pending; persistenceUnverified=true");}
        Try("climate",()=>ClimateState(world.GetOrCreateSystemManaged<ClimateSystem>(),world.GetOrCreateSystemManaged<WindSimulationSystem>(),prefabs,phase,log));
        Try("water",()=>WaterState(manager));
        if(phase=="after-apply")
        {try{NativeNetworkDiagnostics.Check(manager,log);}catch(Exception exception){log.Diagnostic($"NATIVE CONNECTIVITY readback unavailable: {exception}");}}
        if(phase=="after-apply")
        {
            WaterApplied=WaterApplied&&DiagnosticEvidence.AllExpectedPresent(_expectedWater,current["water"] as JArray);
            log.Diagnostic($"WATER READBACK expectedSources={_expectedWater.Count}; nativeSourcesStillMatch={WaterApplied}; downstreamCoverageVerified=false");
            WaterProbeState(world,phase,log,5);
        }
        log.Diagnostic($"SNAPSHOT phase={phase}; terrainSignature={key}; mapSession={ImportSession.Map}; data="+current.ToString(Newtonsoft.Json.Formatting.None));
        var path=Path.Combine(Root,key+".json");
        if(phase=="after-apply")
        {
            var observedAir=current["air"] as JArray;
            var airFound=observedAir==null?0:_expectedAir.Count(expected=>observedAir.Any(actual=>JToken.DeepEquals(expected,actual)));
            AirApplied=AirApplied&&DiagnosticEvidence.AllExpectedPresent(_expectedAir,observedAir)&&_airPendingInitialization==0;
            SurfaceApplied=SurfaceApplied&&_expectedLocalSurface!=null&&_expectedWorldSurface!=null&&(string?)current["surface"]?["local"]?["sha256"]==_expectedLocalSurface&&(string?)current["surface"]?["world"]?["sha256"]==_expectedWorldSurface;
            ClimateApplied=ClimateApplied&&_expectedClimate!=null&&(string?)current["climate"]?["prefab"]==_expectedClimate;
            log.Diagnostic($"DELAYED VERIFICATION expectedCreatedAir={_expectedAir.Count}; expectedAirFound={airFound}; airCreatedMarkersRemaining={_airPendingInitialization}; appliedSurfaceStillMatches={SurfaceApplied}; appliedClimateStillMatches={ClimateApplied}; initializedAirObserved={AirApplied}; functionalFlightsVerified=false");
            current["applied"]=new JObject { ["surface"]=SurfaceApplied,["air"]=AirApplied,["climate"]=ClimateApplied,["water"]=WaterApplied };
            Directory.CreateDirectory(Root);
            // Small metadata only: no image/height buffer is written to the sidecar.
            File.WriteAllText(path,current.ToString(Newtonsoft.Json.Formatting.None));
            log.Diagnostic($"BASELINE stored run={_run}; terrainSignature={key}; appliedSurface={SurfaceApplied}; appliedAir={AirApplied}; appliedClimate={ClimateApplied}; thisIsNotASaveConfirmation=true");
        }
        else
        {
            if(!File.Exists(path)){log.Diagnostic($"RELOAD no-baseline terrainSignature={key}; persistenceUnverified=true");return;}
            var info=new FileInfo(path);if(info.Length>262144)throw new InvalidOperationException("Diagnostic baseline exceeds metadata limit.");
            var previous=JObject.Parse(File.ReadAllText(path));
            if((int?)previous["version"]!=1){log.Diagnostic("RELOAD incompatible-baseline; persistenceUnverified=true");return;}
            foreach(var feature in new[]{"surface","air","climate","water"})
            {
                var applied=(bool?)previous["applied"]?[feature]==true;
                var available=previous[feature]!=null&&current[feature]!=null;
                var match=available&&JToken.DeepEquals(previous[feature],current[feature]);
                log.Diagnostic($"RELOAD feature={feature}; baselineRun={previous["run"]}; baselineUtc={previous["capturedUtc"]}; appliedInBaseline={applied}; readbackAvailable={available}; nativePropertiesMatch={match}; evidence={DiagnosticEvidence.Compare(applied,previous[feature],current[feature])}; identity=terrain-content-not-unique-save-id; entityIdsCompared=false");
            }
        }
        void Try(string feature,Func<JToken> read)
        {try{current[feature]=read();}catch(Exception exception){log.Diagnostic($"READBACK UNAVAILABLE feature={feature}; phase={phase}: {exception}");}}
    }

    private static JObject WaterRecord(float3 position,WaterSourceData source)
        =>new() { ["x"]=Math.Round(position.x,2),["y"]=Math.Round(position.y,2),["z"]=Math.Round(position.z,2),
            ["radius"]=Math.Round(source.m_Radius,2),["depth"]=Math.Round(source.m_Height,2),["modifier"]=source.m_Modifier,["polluted"]=source.m_Polluted,["legacyMode"]=source.m_ConstantDepth };

    private static void WaterProbeState(World world,string phase,ImportLog log,double elapsed)
    {
        try
        {
            var water=world.GetOrCreateSystemManaged<WaterSystem>();
            world.EntityManager.CompleteAllTrackedJobs();
            var native=WaterState(world.EntityManager);
            var flags=System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.Public|System.Reflection.BindingFlags.NonPublic;
            var newMap=typeof(WaterSystem).GetProperty("IsNewMap",flags)?.GetValue(water);
            using var collector=world.EntityManager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<WaterSourceData>()},None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Game.Tools.Temp>()}});
            log.Diagnostic($"WATER SIMULATION phase={phase}; loaded={water.Loaded}; systemEnabled={water.Enabled}; waterSimSpeed={water.WaterSimSpeed}; legacy={water.UseLegacyWaterSources}; cellSize={water.CellSize:R}; newMapSourceStepBlocked={newMap??"unavailable"}; collectorQueryCount={collector.CalculateEntityCount()}; expectedSources={_expectedWater.Count}; allExpectedNativeSourcesPresent={DiagnosticEvidence.AllExpectedPresent(_expectedWater,native)}; elapsedWallSeconds={elapsed:0}");
            // One dependency completion/readback per diagnostic interval, not
            // once per regulator. Probe all bodies; bound individual log examples.
            var depths=water.GetSurfaceData(out var depthDependency);depthDependency.Complete();
            var sampled=new System.Collections.Generic.Dictionary<(int,int),float>();
            float Depth(float3 p)
            {
                var key=((int)Math.Round(p.x*4),(int)Math.Round(p.z*4));
                if(!sampled.TryGetValue(key,out var d))sampled[key]=d=depths.isCreated&&depths.hasDepths?WaterUtils.SampleDepth(ref depths,p):float.NaN;
                return d;
            }
            foreach(var position in WaterSourcePositions.Take(128))
            {
                var depth=Depth(position);
                log.Diagnostic($"WATER SOURCE PROBE phase={phase}; position={position}; depthReadbackMeters={depth:R}; waterSurfaceLevel={(math.isfinite(depth)?position.y+depth:float.NaN):R}; sourcePersistenceAndFlowAreSeparate=true");
            }
            try
            {
                foreach(var cacheName in new[]{"LastFrameSourceCache","CurrentJobSourceCache"})
                {
                var property=typeof(WaterSystem).GetProperty(cacheName,flags);
                if(!(property?.GetValue(water) is NativeList<WaterSourceCache> cache)){log.Diagnostic($"WATER SIM CACHE phase={phase}; cache={cacheName}; unavailable=true");continue;}
                log.Diagnostic($"WATER SIM CACHE phase={phase}; cache={cacheName}; created={cache.IsCreated}; entries={(cache.IsCreated?cache.Length:0)}; coordinateSpace=terrain-relative");
                if(cache.IsCreated)for(var i=0;i<Math.Min(cache.Length,64);i++){var s=cache[i];log.Diagnostic($"WATER SIM CACHE SOURCE index={i}; position={s.m_Position}; radius={s.m_Radius:R}; depth={s.m_Height:R}; mode={s.m_ConstantDepth}; polluted={s.m_Polluted:R}");}
                }
            }
            catch(Exception exception){log.Diagnostic("WATER SIM CACHE unavailable: "+exception.GetBaseException().Message);}
            var read=typeof(WaterSystem).GetMethod("HasWater",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.Public|System.Reflection.BindingFlags.NonPublic,null,new[]{typeof(float3)},null);
            if(read==null)throw new InvalidOperationException("Native HasWater readback is unavailable.");
            // Wetness is separate from source persistence and is not a full-channel/flooding guarantee.
            foreach(var probe in WaterProbes.Take(128))log.Diagnostic($"WATER PROBE phase={phase}; position={probe}; hasNativeWater={read.Invoke(water,new object[]{probe})}; depthReadbackMeters={Depth(probe):R}; elapsedWallSeconds={elapsed:0}; waterSimSpeed={water.WaterSimSpeed}; fullRiverCoverageVerified=false");
            var createdByBody=Enumerable.Range(0,WaterPlans.Count).GroupBy(i=>WaterPlans[i].BodyId).ToDictionary(g=>g.Key,g=>g.ToArray());
            var bodyRecords=new System.Collections.Generic.List<object>();
            var remainingProbeRecords=32768;
            var plannedBodyCount=0;
            foreach(var body in PlannedWater.GroupBy(p=>p.BodyId))
            {
                plannedBodyCount++;
                var plans=body.ToArray();var plan=plans[0];
                var indexes=createdByBody.TryGetValue(body.Key,out var created)?created:Array.Empty<int>();
                var probes=plans.SelectMany(p=>p.ChannelProbes.Count>0?p.ChannelProbes:new[]{p.DownstreamProbe}).GroupBy(p=>((int)Math.Round(p.x*4),(int)Math.Round(p.z*4))).Select(g=>g.First()).ToArray();
                var banks=plans.SelectMany(p=>p.BankProbes).GroupBy(p=>((int)Math.Round(p.x*4),(int)Math.Round(p.z*4))).Select(g=>g.First()).ToArray();
                var wet=probes.Count(p=>Depth(p)>.05f);var flooded=banks.Count(p=>Depth(p)>.1f);
                var sourceDepths=indexes.Select(i=>Depth(WaterSourcePositions[i])).Where(math.isfinite).ToArray();
                var levels=indexes.Select(i=>WaterSourcePositions[i].y+Depth(WaterSourcePositions[i])).Where(math.isfinite).OrderBy(h=>h).ToArray();
                var level=levels.Length>0?levels[levels.Length/2]:float.NaN;
                var hadPrevious=WaterObservations.TryGetValue(body.Key,out var previous);
                var dryGap=0f;foreach(var p in plans){var points=p.ChannelProbes;for(var j=1;j<points.Count;j++)if(Depth(points[j-1])<=.05f||Depth(points[j])<=.05f)dryGap=Math.Max(dryGap,math.distance(points[j-1].xz,points[j].xz));}
                log.Diagnostic($"WATER BODY RESULT body={body.Key}; kind={plan.BodyKind}; controls={indexes.Length}/{plans.Length}; missingPlannedControls={plans.Length-indexes.Length}; minimumSourceDepth={(sourceDepths.Length>0?sourceDepths.Min():float.NaN):R}; wetDownstreamProbes={wet}/{probes.Length}; floodedBankProbes={flooded}/{banks.Length}; largestPotentialDryProbeGapMeters={dryGap:R}; medianObservedLevel={level:R}; previousWetProbes={(hadPrevious?previous.wet:-1)}; medianLevelChangeMeters={(hadPrevious?level-previous.level:float.NaN):R}; elapsedWallSeconds={elapsed:0}; singleSurfaceSnapshot=true; fullRiverCoverageVerified=false; simulationAdvanceVerified=false; sourceCausedWetnessVerified=false");
                WaterObservations[body.Key]=(wet,probes.Length,flooded,level);
                if(bodyRecords.Count<4096)
                {
                    var channelRecords=Math.Min(Math.Min(4096,probes.Length),remainingProbeRecords);remainingProbeRecords-=channelRecords;
                    var bankRecords=Math.Min(Math.Min(4096,banks.Length),remainingProbeRecords);remainingProbeRecords-=bankRecords;
                    bodyRecords.Add(new {
                    body=body.Key,kind=plan.BodyKind,phase,elapsedWallSeconds=elapsed,waterSimSpeed=water.WaterSimSpeed,
                    plannedControls=plans.Length,createdControls=indexes.Length,wetProbes=wet,floodedBankProbes=flooded,
                    channel=probes.Take(channelRecords).Select(p=>new {position=DiagnosticArtifacts.Point(p),depthMeters=Depth(p)}).ToArray(),
                    banks=banks.Take(bankRecords).Select(p=>new {position=DiagnosticArtifacts.Point(p),depthMeters=Depth(p)}).ToArray(),
                    channelProbeCount=probes.Length,bankProbeCount=banks.Length,probesTruncated=probes.Length>channelRecords||banks.Length>bankRecords,
                    sourceCausedWetnessVerified=false,simulationAdvanceVerified=false
                    });
                }
                if(flooded>0)log.Warn($"Water containment not verified for body {body.Key}: {flooded} outer-bank probes are wet. No automatic source increase is permitted; check neighbouring bodies and sea level.");
                if(elapsed>=179&&(sourceDepths.Length==0||sourceDepths.Min()<=.05f||wet<probes.Length))
                    log.Warn($"Water coverage not verified for body {body.Key}: {wet}/{probes.Length} channel samples wet after {elapsed:0} seconds. Native source existence and wall time do not prove simulation advancement; inspect WATER SIMULATION/CACHE diagnostics.");
            }
            log.DiagnosticRecords($"water-observations-{(int)elapsed}s.ndjson",bodyRecords);
            log.Diagnostic($"WATER OBSERVATION ARTIFACT bodiesRecorded={bodyRecords.Count}; bodiesPlanned={plannedBodyCount}; positionedDepths=true; bodyRecordLimit=4096; intervalProbeRecordLimit=32768; unchangedReadOnlySampling=true");
        }
        catch(Exception exception){log.Diagnostic("WATER PROBE unavailable: "+exception.GetBaseException().Message);}
    }

    private static JArray WaterState(EntityManager manager)
    {
        using var query=manager.CreateEntityQuery(new EntityQueryDesc { All=new[]{ComponentType.ReadOnly<WaterSourceData>(),ComponentType.ReadOnly<Transform>()},None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Game.Tools.Temp>()} });
        using var entities=query.ToEntityArray(Allocator.Temp);
        if(entities.Length>20000)throw new InvalidOperationException("More than 20,000 native water sources; bounded persistence readback unavailable.");
        return new JArray(entities.Select(entity=>WaterRecord(manager.GetComponentData<Transform>(entity).m_Position,manager.GetComponentData<WaterSourceData>(entity))).OrderBy(v=>v.ToString(Newtonsoft.Json.Formatting.None),StringComparer.Ordinal));
    }

    private static JObject TextureSignature(Texture? target)
    {
        if(target is RenderTexture uninitialized&&!uninitialized.IsCreated())return new JObject{["initialized"]=false,["width"]=uninitialized.width,["height"]=uninitialized.height,["format"]=uninitialized.format.ToString(),["meaning"]="Native editor paint texture has not been allocated; base terrain only."};
        if(!(target is RenderTexture render)||!render.IsCreated()||render.graphicsFormat!=UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8_UNorm||render.width<=0||render.height<=0||(long)render.width*render.height>16_777_216)
            throw new InvalidOperationException($"Native paint texture unavailable or unsupported: type={target?.GetType().Name??"null"}; dimensions={target?.width}x{target?.height}; format={(target is RenderTexture rt?rt.format.ToString():"not-render-texture")}.");
        var cpu=new Texture2D(render.width,render.height,TextureFormat.RG16,false,true);var old=RenderTexture.active;
        try
        {
            RenderTexture.active=render;cpu.ReadPixels(new Rect(0,0,render.width,render.height),0,0,false);var pixels=cpu.GetRawTextureData();
            if(pixels.Length!=(long)render.width*render.height*2)throw new InvalidOperationException("Native RG16 diagnostic readback byte count changed.");
            var counts=new long[2];for(var i=0;i<pixels.Length;i+=2){if(pixels[i]>0)counts[0]++;if(pixels[i+1]>0)counts[1]++;}
            return new JObject { ["width"]=render.width,["height"]=render.height,["format"]=render.format.ToString(),["encoding"]="nativeRG16-not-four-brush-weights",["sha256"]=PixelHash(pixels),["nonzeroEncodedChannels"]=new JArray(counts) };
        }
        finally{RenderTexture.active=old;UnityEngine.Object.Destroy(cpu);}
    }

    private static JArray AirState(EntityManager manager,PrefabSystem prefabs,NameSystem names,string phase,ImportLog log)
    {
        _airPendingInitialization=0;
        using var query=manager.CreateEntityQuery(new EntityQueryDesc { All=new[]{ComponentType.ReadOnly<AirplaneStop>(),ComponentType.ReadOnly<Game.Objects.OutsideConnection>(),ComponentType.ReadOnly<Transform>(),ComponentType.ReadOnly<PrefabRef>()},None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Game.Tools.Temp>()} });
        using var entities=query.ToEntityArray(Allocator.Temp);
        if(entities.Length>256)throw new InvalidOperationException("More than 256 airplane connections; refusing a truncated persistence claim.");
        var result=new JArray();foreach(var entity in entities)
        {
            var position=manager.GetComponentData<Transform>(entity).m_Position;
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;
            if(!prefabs.TryGetPrefab<PrefabBase>(prefab,out var resolved)||resolved==null)throw new InvalidOperationException("Native airplane prefab reference cannot be resolved; no persistence claim.");
            var prefabName=resolved.name;
            names.TryGetCustomName(entity,out var name);
            var takeoff=manager.HasComponent<Game.Routes.TakeoffLocation>(entity);
            var created=manager.HasComponent<Created>(entity);
            if(created)_airPendingInitialization++;
            log.Diagnostic($"AIR STATE phase={phase}; name={name}; prefab={prefabName}; position={position}; airplaneStop=true; outsideConnection=true; takeoffLocation={takeoff}; createdMarkerStillPresent={created}; takeoffFlags="+(takeoff?manager.GetComponentData<Game.Routes.TakeoffLocation>(entity).m_Flags.ToString():"missing")+"; flightsVerified=false");
            result.Add(AirRecord(name??"",prefabName,position,takeoff));
        }
        return new JArray(result.OrderBy(v=>v.ToString(Newtonsoft.Json.Formatting.None),StringComparer.Ordinal));
    }

    private static JObject AirRecord(string name,string prefab,float3 position,bool takeoff)
        =>new() { ["name"]=name,["prefab"]=prefab,["x"]=Math.Round(position.x,2),["y"]=Math.Round(position.y,2),["z"]=Math.Round(position.z,2),["airplaneStop"]=true,["outsideConnection"]=true,["takeoffLocation"]=takeoff };

    public static JObject ClimateState(ClimateSystem climate,WindSimulationSystem wind,PrefabSystem prefabs,string phase,ImportLog log)
    {
        var entity=climate.currentClimate;
        if(!prefabs.TryGetPrefab<ClimatePrefab>(entity,out var prefab)||prefab==null)throw new InvalidOperationException("Native climate prefab cannot be resolved; no persistence claim.");
        var name=prefab.name;
        var pressure=typeof(WindSimulationSystem).GetProperty("m_ConstantPressure",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.Public|System.Reflection.BindingFlags.NonPublic)?.GetValue(wind);
        var flow=wind.constantWind;
        if(!(pressure is float nativePressure)||!math.isfinite(nativePressure)||!math.all(math.isfinite(flow)))throw new InvalidOperationException("Native wind/pressure readback is unavailable or non-finite; no persistence claim.");
        log.Diagnostic($"CLIMATE STATE phase={phase}; prefab={name}; windVector={flow}; nativeWindMagnitude={math.length(flow):R}; pressure={pressure}; temperature={climate.temperature.value:R}; precipitation={climate.precipitation.value:R}; cloudiness={climate.cloudiness.value:R}; seasonTemperature={climate.seasonTemperature:R}; seasonPrecipitation={climate.seasonPrecipitation:R}; seasonCloudiness={climate.seasonCloudiness:R}; weatherClassification={climate.classification}; weatherIsTimeDependent=true");
        // Dynamic weather is reported, not expected to remain identical across elapsed time.
        return new JObject { ["prefab"]=name,["windX"]=Math.Round(flow.x,4),["windZ"]=Math.Round(flow.y,4),["pressure"]=pressure is float p?new JValue(Math.Round(p,3)):JValue.CreateNull() };
    }
}
