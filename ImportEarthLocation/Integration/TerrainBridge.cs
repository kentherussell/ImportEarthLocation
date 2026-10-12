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
