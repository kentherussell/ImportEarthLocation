using System;
using System.Linq;
using System.Reflection;
using Game.Simulation;
using ImportEarthLocation.Core;
using Unity.Mathematics;
using UnityEngine;

namespace ImportEarthLocation.Integration;

public sealed class TerrainBridge
{
    private readonly TerrainSystem _terrainSystem;
    private readonly WaterSystem _waterSystem;
    private readonly MethodInfo? _setTerrainProperties;
    private readonly MethodInfo? _replaceHeightmap;
    private readonly MethodInfo? _setHeightmap;
    private readonly MethodInfo? _setWorldHeightmap;
    private readonly MethodInfo? _finalizeTerrain;
    private readonly MethodInfo? _areaChanged,_updateCascades;
    private readonly FieldInfo? _heightChanged,_heightReaders,_downsampleReaders;
    private Restamp? _restamp;
    private sealed record Restamp(Texture2D Backup,int X,int Z,int Frame,ImportRequest Request,Processing.NetworkRoadbed.Patch Patch,float Scale);

    public TerrainBridge(TerrainSystem terrainSystem, WaterSystem waterSystem)
    {
        _terrainSystem = terrainSystem;
        _waterSystem = waterSystem;
        _setTerrainProperties = typeof(TerrainSystem)
            .GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .FirstOrDefault(method =>
                method.Name.IndexOf("SetTerrainProperties", StringComparison.OrdinalIgnoreCase) >= 0 &&
                method.GetParameters().Length == 1 &&
                method.GetParameters()[0].ParameterType == typeof(float2));
        _replaceHeightmap = typeof(TerrainSystem).GetMethod(
            "ReplaceHeightmap",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            types: new[] { typeof(Texture2D) },
            modifiers: null);
        var flags=BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic;
        _setHeightmap=typeof(TerrainSystem).GetMethod("SetHeightmap",flags,null,new[]{typeof(Texture2D)},null);
        _setWorldHeightmap=typeof(TerrainSystem).GetMethod("SetWorldHeightmap",flags,null,new[]{typeof(Texture2D),typeof(bool)},null);
        _finalizeTerrain=typeof(TerrainSystem).GetMethod("FinalizeTerrainData",flags,null,new[]{typeof(Texture2D),typeof(Texture2D),typeof(float2),typeof(float2),typeof(float2),typeof(float2),typeof(float2),typeof(float2)},null);
        _areaChanged=typeof(TerrainSystem).GetMethod("OnAreaChanged",flags,null,new[]{typeof(Colossal.Mathematics.Bounds2).MakeByRefType()},null);
        _updateCascades=typeof(TerrainSystem).GetMethod("UpdateCascades",flags,null,new[]{typeof(bool),typeof(bool)},null);
        _heightChanged=typeof(TerrainSystem).GetField("m_HeightMapChanged",flags);
        _heightReaders=typeof(TerrainSystem).GetField("m_CPUHeightReaders",flags);
        _downsampleReaders=typeof(TerrainSystem).GetField("m_CPUDownSampleHeightReaders",flags);
    }

    public bool BeginRestamp(Processing.NetworkRoadbed.Patch patch,ImportRequest request,float scale,ImportLog log)
    {
        // Require the native update hooks and resources without restricting
        // engine build IDs. Publication still requires sampled CPU readback.
        if(_areaChanged==null||_updateCascades==null||_heightChanged==null||_heightReaders==null||_downsampleReaders==null||
            _terrainSystem.freezeCascadeUpdates||_restamp!=null||_terrainSystem.heightmap is not RenderTexture target)
        {log.Diagnostic("LOCAL TERRAIN unavailable native dirty-region contract; original unit retained");return false;}
        if(!ImportSession.Matches(request.MapSessionId,request.Generation))return false;
        _terrainSystem.GetHeightData(true);
        CompleteReaders();
        var data=_terrainSystem.GetHeightData(false);var bounds=TerrainUtils.GetBounds(ref data);
        var half=request.MapSize.WidthMeters/2;var tolerance=request.MapSize.WidthMeters/(target.width-1)*2;
        if(Math.Abs(bounds.min.x+half)>tolerance||Math.Abs(bounds.max.x-half)>tolerance||Math.Abs(bounds.min.z+half)>tolerance||Math.Abs(bounds.max.z-half)>tolerance)return false;
        int X(float x)=>(int)math.clamp(Math.Floor((x-bounds.min.x)/(bounds.max.x-bounds.min.x)*(target.width-1)),0,target.width-1);
        int Z(float z)=>(int)math.clamp(Math.Floor((z-bounds.min.z)/(bounds.max.z-bounds.min.z)*(target.height-1)),0,target.height-1);
        var x0=X(patch.Minimum.x);var z0=Z(patch.Minimum.y);var x1=X(patch.Maximum.x);var z1=Z(patch.Maximum.y);
        var width=x1-x0+1;var height=z1-z0+1;var samples=new ushort[width*height];
        for(var z=0;z<height;z++)for(var x=0;x<width;x++)
        {
            var wx=math.lerp(bounds.min.x,bounds.max.x,(x0+x)/(float)(target.width-1));
            var wz=math.lerp(bounds.min.z,bounds.max.z,(z0+z)/(float)(target.height-1));
            samples[z*width+x]=(ushort)Math.Round(patch.Encoded(wx,wz));
        }
        var backup=new Texture2D(width,height,TextureFormat.R16,false,true);var active=RenderTexture.active;Texture2D? upload=null;
        try
        {
            RenderTexture.active=target;backup.ReadPixels(new Rect(x0,z0,width,height),0,0,false);backup.Apply(false,false);
            _restamp=new(backup,x0,z0,Time.frameCount,request,patch,scale);
            upload=new Texture2D(width,height,TextureFormat.R16,false,true);var bytes=new byte[samples.Length*2];Buffer.BlockCopy(samples,0,bytes,0,bytes.Length);
            upload.LoadRawTextureData(bytes);upload.Apply(false,false);
            Graphics.CopyTexture(upload,0,0,0,0,width,height,target,0,0,x0,z0);
            PublishDirty(patch);
            log.Diagnostic($"LOCAL TERRAIN published nativeRect={x0},{z0},{width},{height}; outsidePixelsPreserved=true; CPUReadbackPending=true; fullMapReplacement=false");
            return true;
        }
        catch
        {
            if(_restamp!=null)CancelRestamp(log);else UnityEngine.Object.Destroy(backup);
            throw;
        }
        finally{RenderTexture.active=active;if(upload!=null)UnityEngine.Object.Destroy(upload);}
    }

    private void CompleteReaders()
    {
        foreach(var field in new[]{_heightReaders,_downsampleReaders})
            if(field?.GetValue(_terrainSystem) is Unity.Jobs.JobHandle handle){handle.Complete();field.SetValue(_terrainSystem,default(Unity.Jobs.JobHandle));}
    }

    private void PublishDirty(Processing.NetworkRoadbed.Patch patch)
    {
        var area=new Colossal.Mathematics.Bounds2(patch.Minimum,patch.Maximum);
        _waterSystem.TerrainWillChangeFromBrush(area);
        _areaChanged!.Invoke(_terrainSystem,new object[]{area});
        // Native OnUpdate runs these same cascade/render stages. RenderCascades
        // records the dirty GPU extent and triggers asynchronous CPU publication.
        _updateCascades!.Invoke(_terrainSystem,new object[]{false,false});
        _terrainSystem.RenderCascades();
    }

    public bool PollRestamp(ImportLog log,out bool published)
    {
        published=false;var operation=_restamp;if(operation==null)return true;
        if(!ImportSession.Matches(operation.Request.MapSessionId,operation.Request.Generation)){CancelRestamp(log);return true;}
        var frames=Time.frameCount-operation.Frame;
        if(frames<2||_heightChanged?.GetValue(_terrainSystem) is true)
        {
            if(frames<180)return false;
            log.Diagnostic("LOCAL TERRAIN publication timeout; restoring the owned patch, not submitting revised geometry");CancelRestamp(log);return true;
        }
        var data=_terrainSystem.GetHeightData(false);var maximum=0f;var samples=0;
        for(var z=0;z<3;z++)for(var x=0;x<3;x++)
        {
            var point=math.lerp(operation.Patch.Minimum,operation.Patch.Maximum,new float2(x/2f,z/2f));
            var actual=TerrainUtils.SampleHeight(ref data,new float3(point.x,0,point.y));
            if(!math.isfinite(actual)){CancelRestamp(log);return true;}
            var expected=(float)(operation.Patch.Encoded(point.x,point.y)*operation.Scale/65535d);
            maximum=Math.Max(maximum,Math.Abs(actual-expected));samples++;
        }
        log.Diagnostic($"LOCAL TERRAIN readback samples={samples}; maximumDifferenceMeters={maximum:R}; frames={frames}; CPUAvailable=true; targetMatched={maximum<=Math.Max(.25f,operation.Scale/65535f*4)}; sampledNotContinuous=true; nativeRoadAdjustmentsMayDiffer=true");
        UnityEngine.Object.Destroy(operation.Backup);_restamp=null;published=true;return true;
    }

    public void CancelRestamp(ImportLog? log)
    {
        var operation=_restamp;if(operation==null)return;
        try
        {
            if(ImportSession.Matches(operation.Request.MapSessionId,operation.Request.Generation)&&_terrainSystem.heightmap is RenderTexture target)
            {
                _terrainSystem.GetHeightData(true);CompleteReaders();
                Graphics.CopyTexture(operation.Backup,0,0,0,0,operation.Backup.width,operation.Backup.height,target,0,0,operation.X,operation.Z);
                PublishDirty(operation.Patch);
                // A retry must not sample the cancelled candidate's CPU data.
                _terrainSystem.GetHeightData(true);
                log?.Diagnostic("LOCAL TERRAIN rollback published; original unapplied unit retained; outside pixels unchanged");
            }
        }
        finally{UnityEngine.Object.Destroy(operation.Backup);_restamp=null;}
    }

    public bool CanApply(out string reason)
    {
        if (!IsTerrainLoaded()) { reason = "Editor terrain resources are not ready. Finish loading a map before preparing an import."; return false; }
        if (!_waterSystem.Loaded) { reason = "WaterSystem is not loaded; open a map in the editor first."; return false; }
        if (_replaceHeightmap == null) { reason = "The current CS2 TerrainSystem no longer exposes the expected ReplaceHeightmap hook."; return false; }
        if (_setTerrainProperties == null) { reason = "The current CS2 TerrainSystem no longer exposes the expected terrain-property hook."; return false; }
        if(_setHeightmap==null||_setWorldHeightmap==null||_finalizeTerrain==null){reason="Native terrain replacement/transform recovery hooks are unavailable; map preserved.";return false;}
        reason = "";
        return true;
    }

    private bool IsTerrainLoaded()
    {
        // m_Loaded is a one-update cascade-refresh flag, NOT persistent map readiness.
        // Verify the render heightmap and the public CPU terrain data instead.
        var heightmap = _terrainSystem.heightmap;
        if (heightmap == null || heightmap.width <= 0 || heightmap.height <= 0) return false;
        var data = _terrainSystem.GetHeightData(false);
        return data.heights.IsCreated && data.heights.Length > 0 && data.resolution.x > 2 && data.resolution.z > 2;
    }

    public void Apply(PreparedImport prepared,ImportLog? log=null)
    {
        if (!CanApply(out var reason)) throw new InvalidOperationException(reason);
        if (prepared.Heightmap.Length != Processing.TerrainBuilder.Resolution * Processing.TerrainBuilder.Resolution)
            throw new InvalidOperationException("Prepared heightmap dimensions are invalid.");

        // Snapshot the actual playable texture, not the CPU cascade (which may
        // include backdrop data at a different resolution). Capture before any
        // mutation or old-network deletion, and restore the same transforms.
        var oldScale=_terrainSystem.heightScaleOffset;var oldSea=_waterSystem.SeaLevel;
        var mapCorner=_terrainSystem.playableOffset;var mapSize=_terrainSystem.playableArea;
        var worldCorner=_terrainSystem.worldOffset;var worldSize=_terrainSystem.worldSize;var worldRange=_terrainSystem.worldHeightMinMax;
        var oldWorldAsset=_terrainSystem.worldMapAsset;var oldCulling=_waterSystem.UseActiveCellsCulling;
        var backup=Capture(_terrainSystem.heightmap);Texture2D? worldBackup=null;
        Texture2D? texture=null;var mutationStarted=false;
        try
        {
        if(_terrainSystem.worldHeightmap!=null)worldBackup=Capture(_terrainSystem.worldHeightmap);
        // Keep the existing native texture resolution. Changing it can destroy
        // the world/backdrop heightmap, even if map dimensions are unchanged.
        texture = new Texture2D(
            backup.width,
            backup.height,
            TextureFormat.R16,
            mipChain: false,
            linear: true)
        { name = $"{Mod.Id}_Heightmap" };

        var data=_terrainSystem.GetHeightData(false);var nativeBounds=TerrainUtils.GetBounds(ref data);
        var half=(float)prepared.Request.MapSize.WidthMeters*.5f;var registrationTolerance=prepared.Request.MapSize.WidthMeters/Math.Max(1,backup.width-1)*2;
        var registered=Math.Abs(nativeBounds.min.x+half)<=registrationTolerance&&Math.Abs(nativeBounds.max.x-half)<=registrationTolerance&&Math.Abs(nativeBounds.min.z+half)<=registrationTolerance&&Math.Abs(nativeBounds.max.z-half)<=registrationTolerance;
        var nativeSamples=registered?new ushort[backup.width*backup.height]:Resample(prepared.Heightmap,backup.width,backup.height);
        if(registered)for(var z=0;z<backup.height;z++)for(var x=0;x<backup.width;x++)
        {var wx=math.lerp(nativeBounds.min.x,nativeBounds.max.x,x/(float)(backup.width-1));var wz=math.lerp(nativeBounds.min.z,nativeBounds.max.z,z/(float)(backup.height-1));
            nativeSamples[z*backup.width+x]=(ushort)Math.Round(Processing.TerrainSampling.Encoded(prepared.Heightmap,Processing.TerrainBuilder.Resolution,wx/(2*half)+.5,wz/(2*half)+.5));}
        log?.Diagnostic($"TERRAIN REGISTRATION nativeSampleBoundsMin={nativeBounds.min}; nativeSampleBoundsMax={nativeBounds.max}; worldSampleResamplingApplied={registered}; toleranceMeters={registrationTolerance:R}; sameResolutionDoesNotImplySameTransform=true");
        log?.Diagnostic($"TERRAIN GRID interpolation=corner-aligned-bilinear-v1; preparedResolution={Processing.TerrainBuilder.Resolution}; nativeWidth={backup.width}; nativeHeight={backup.height}; requestedWidthMeters={prepared.Request.MapSize.WidthMeters:R}; nativeAreaX={mapSize.x:R}; nativeAreaZ={mapSize.y:R}; nativeOffsetX={mapCorner.x:R}; nativeOffsetZ={mapCorner.y:R}; encodedQuantizationMeters={prepared.HeightScaleMeters / ushort.MaxValue:R}; nativeResamplingRequired={backup.width != Processing.TerrainBuilder.Resolution || backup.height != Processing.TerrainBuilder.Resolution}; gridEquivalenceVerified=false");
        var bytes = new byte[nativeSamples.Length * sizeof(ushort)];
        Buffer.BlockCopy(nativeSamples, 0, bytes, 0, bytes.Length);
        texture.LoadRawTextureData(bytes);
        // ReplaceHeightmap calls GetRawTextureData<ushort> to populate CPU heights.
        texture.Apply(updateMipmaps: false, makeNoLongerReadable: false);

            mutationStarted=true;
            // ReplaceHeightmap resets map/world corners to vanilla defaults.
            // Use its verified internal stages with the loaded map transforms.
            _setHeightmap!.Invoke(_terrainSystem,new object[]{texture});
            RestoreWorldIfDestroyed();
            Finalize(texture,new float2(prepared.HeightScaleMeters,0));
            _waterSystem.SeaLevel = WaterLevels.Effective(prepared.SeaLevelMeters);
            _waterSystem.UseActiveCellsCulling=false;_waterSystem.TerrainWillChange();
            if(math.any(math.abs(_terrainSystem.playableArea-mapSize)>.01f)||math.any(math.abs(_terrainSystem.playableOffset-mapCorner)>.01f)||math.any(math.abs(_terrainSystem.heightScaleOffset-new float2(prepared.HeightScaleMeters,0))>.001f))
                throw new InvalidOperationException("Native terrain transforms did not match the approved map.");
            log?.Diagnostic($"TERRAIN APPLY transformPreserved=true; mapSize={mapSize}; mapCorner={mapCorner}; scale={prepared.HeightScaleMeters:R}; nativeResolution={backup.width}x{backup.height}; backupCaptured=true; networkDeletionNotStarted=true; readbackNotVisualAcceptance=true");
        }
        catch(Exception error)
        {
            if(!mutationStarted)throw;
            try
            {
                _setHeightmap!.Invoke(_terrainSystem,new object[]{backup});RestoreWorldIfDestroyed();Finalize(backup,oldScale);
                _terrainSystem.worldMapAsset=oldWorldAsset;_waterSystem.SeaLevel=oldSea;_waterSystem.UseActiveCellsCulling=oldCulling;_waterSystem.TerrainWillChange();
                log?.Diagnostic("TERRAIN ROLLBACK restored-playable-texture-and-transforms; oldNetworkDeletionNotStarted=true; fullGenerationRollback=false");
            }
            catch(Exception recovery)
            {
                log?.Diagnostic($"TERRAIN ROLLBACK FAILED original={error}; restore={recovery}; oldNetworksNotDeleted=true");
                throw new InvalidOperationException("Native terrain replacement and restoration failed. Old networks were not deleted; reopen the saved map before another import.",error);
            }
            throw new InvalidOperationException("Native terrain replacement failed; prior terrain, transforms and sea level were restored. Old networks were not deleted.",error);
        }
        finally
        {
            if(texture!=null)UnityEngine.Object.Destroy(texture);UnityEngine.Object.Destroy(backup);if(worldBackup!=null)UnityEngine.Object.Destroy(worldBackup);
        }
        void RestoreWorldIfDestroyed()
        {
            if(worldBackup==null||_terrainSystem.worldHeightmap!=null)return;
            // true makes an owned GPU copy. false would retain our temporary CPU
            // backup as the live world map and destroy it in the finally block.
            _setWorldHeightmap!.Invoke(_terrainSystem,new object[]{worldBackup,true});
            if(_terrainSystem.worldHeightmap==null)throw new InvalidOperationException("Native world terrain could not be restored.");
            _terrainSystem.worldMapAsset=oldWorldAsset;
        }
        void Finalize(Texture2D map,float2 scale)
        {_finalizeTerrain!.Invoke(_terrainSystem,new object?[]{map,worldBackup,scale,mapCorner,mapSize,worldCorner,worldSize,worldRange});}
    }

    private static ushort[] Resample(ushort[] input,int width,int height)
    {
        return Processing.TerrainSampling.Resample(input,width,height);
    }

    private static Texture2D Capture(Texture source)
    {
        if(source==null||source.width<=0||source.height<=0||(long)source.width*source.height>67_108_864)
            throw new InvalidOperationException("Prior terrain cannot be captured safely; current map preserved.");
        var cpu=new Texture2D(source.width,source.height,TextureFormat.R16,false,true);var old=RenderTexture.active;RenderTexture? temporary=null;
        try
        {
            var render=source as RenderTexture;
            if(render==null)
            {temporary=RenderTexture.GetTemporary(source.width,source.height,0,RenderTextureFormat.R16,RenderTextureReadWrite.Linear);Graphics.Blit(source,temporary);render=temporary;}
            if(!render.IsCreated())throw new InvalidOperationException("Prior terrain GPU texture is unavailable; current map preserved.");
            RenderTexture.active=render;cpu.ReadPixels(new Rect(0,0,source.width,source.height),0,0,false);cpu.Apply(false,false);
            if(cpu.GetRawTextureData().Length!=(long)source.width*source.height*2)throw new InvalidOperationException("Prior terrain R16 snapshot is invalid; current map preserved.");
            return cpu;
        }
        catch{UnityEngine.Object.Destroy(cpu);throw;}
        finally{RenderTexture.active=old;if(temporary!=null)RenderTexture.ReleaseTemporary(temporary);}
    }
}
