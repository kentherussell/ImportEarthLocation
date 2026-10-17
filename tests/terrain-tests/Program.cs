using Game.Simulation;
using ImportEarthLocation.Integration;
using UnityEngine;

static class Program
{
    static void Check(bool value, string message) { if (!value) throw new Exception(message); }
    static void Main()
    {
        var terrain = new TerrainSystem();
        var water = new WaterSystem { Loaded = true };
        var bridge = new TerrainBridge(terrain, water);
        Check(!bridge.BeginRestamp(new(new(0),new(1)),new(),100,new()),"Unverified stand-in engine MVID allowed native restamping");
        // Deliberately no m_Loaded field: readiness must use the real resource APIs.
        Check(bridge.CanApply(out var reason), "Loaded terrain falsely rejected: " + reason);
        terrain.heightmap = null;
        Check(!bridge.CanApply(out reason) && reason.Contains("terrain resources"), "Missing heightmap accepted");
        terrain.heightmap = new Texture { width = 4096, height = 4096 };
        terrain.Data.heights = new HeightArray { IsCreated = false, Length = 0 };
        Check(!bridge.CanApply(out _), "Uninitialized CPU data accepted");
        terrain.Data.heights = new HeightArray { IsCreated = true, Length = 16 };
        terrain.Data.resolution = new Unity.Mathematics.int3 { x = 2, z = 2 };
        Check(!bridge.CanApply(out _), "Game's fallback/unready resolution accepted");
        terrain.Data.resolution = new Unity.Mathematics.int3 { x = 4, z = 4 };
        Check(bridge.CanApply(out _), "Ready terrain still rejected after reload");
        water.Loaded = false;
        Check(!bridge.CanApply(out reason) && reason.Contains("WaterSystem"), "Unloaded water accepted");
        water.Loaded = true;
        Check(bridge.CanApply(out _), "Ready reloaded map rejected");
        var oldArea=terrain.playableArea;var oldCorner=terrain.playableOffset;
        bridge.Apply(new ImportEarthLocation.Core.PreparedImport { SeaLevelMeters = 40, HeightScaleMeters=100 });
        Check(water.SeaLevel == 41, "Applied water was not exactly 1m above nominal sea level");
        Check(terrain.playableArea.Equals(oldArea)&&terrain.playableOffset.Equals(oldCorner),"Replacement changed approved map transforms");
        terrain.FailFinalizeOnce=true;var priorScale=terrain.heightScaleOffset;
        try{bridge.Apply(new ImportEarthLocation.Core.PreparedImport{SeaLevelMeters=60,HeightScaleMeters=200});throw new Exception("Injected terrain failure ignored");}catch(InvalidOperationException e){Check(e.Message.Contains("restored"),"Terrain failure did not restore prior state");}
        Check(terrain.heightScaleOffset.Equals(priorScale)&&water.SeaLevel==41&&terrain.playableArea.Equals(oldArea),"Terrain rollback lost sea level or transforms");
        Console.WriteLine("PASS: resource readiness/reload, transform-preserving orchestration and injected finalization rollback; simulated hooks only, actual GPU/native contracts checked separately.");
    }
}
namespace UnityEngine
{
    public class Object { public static void Destroy(Object value) { } }
    public class Texture : Object { public int width, height; public byte[] Raw=Array.Empty<byte>(); }
    public enum TextureFormat { R16 }
    public class Texture2D : Texture
    {
        public string name = "";
        public Texture2D(int width, int height, TextureFormat format, bool mipChain, bool linear) { this.width = width; this.height = height; }
        public void LoadRawTextureData(byte[] bytes) { Raw=bytes; }
        public void ReadPixels(Rect rect,int x,int y,bool mip) { Raw=(byte[])RenderTexture.active!.Raw.Clone(); }
        public byte[] GetRawTextureData()=>Raw;
        public void Apply(bool updateMipmaps, bool makeNoLongerReadable) { }
    }
    public readonly record struct Rect(float x,float y,float width,float height);
    public enum RenderTextureFormat { R16 }
    public enum RenderTextureReadWrite { Linear }
    public class RenderTexture : Texture
    {
        public static RenderTexture? active;
        public bool IsCreated()=>true;
        public static RenderTexture GetTemporary(int w,int h,int depth,RenderTextureFormat format,RenderTextureReadWrite read)=>new(){width=w,height=h,Raw=new byte[w*h*2]};
        public static void ReleaseTemporary(RenderTexture value){}
    }
    public static class Graphics
    {
        public static void Blit(Texture source,Texture target)=>target.Raw=source.Raw.Length==source.width*source.height*2?source.Raw.ToArray():new byte[source.width*source.height*2];
        public static void CopyTexture(Texture source,int sourceElement,int sourceMip,int sourceX,int sourceY,int width,int height,Texture target,int targetElement,int targetMip,int targetX,int targetY)
            =>throw new NotSupportedException("Native restamp GPU calls must not execute in this readiness harness");
    }
    public static class Time { public static int frameCount=>0; }
}
namespace Game.Simulation
{
    public struct HeightArray { public bool IsCreated; public int Length; }
    public struct TerrainHeightData { public HeightArray heights; public Unity.Mathematics.int3 resolution; }
    public readonly record struct TerrainBounds(Unity.Mathematics.float3 min, Unity.Mathematics.float3 max);
    public static class TerrainUtils
    {
        public static float SampleHeight(ref TerrainHeightData data,Unity.Mathematics.float3 point)=>throw new NotSupportedException("Native CPU publication is not simulated");
        public static TerrainBounds GetBounds(ref TerrainHeightData data) =>
            new(new(-28672, 0, -28672), new(28672, 1000, 28672));
    }
    public class TerrainSystem
    {
        public Texture? heightmap = new Texture { width = 8, height = 8 };
        public Texture? worldHeightmap=new Texture{width=8,height=8};
        public Unity.Mathematics.float2 heightScaleOffset=new(80,0),playableOffset=new(-28672,-28672),playableArea=new(57344,57344),worldOffset=new(-57344,-57344),worldSize=new(114688,114688),worldHeightMinMax=new(0,1000);
        public object? worldMapAsset;
        public bool FailFinalizeOnce;
        public bool freezeCascadeUpdates=>false;
        public void RenderCascades()=>throw new NotSupportedException("Native cascades are not simulated");
        public TerrainHeightData Data = new() { heights = new() { IsCreated = true, Length = 16 }, resolution = new() { x = 4, z = 4 } };
        public TerrainHeightData GetHeightData(bool sync) => Data;
        public void SetTerrainProperties(Unity.Mathematics.float2 value) { }
        public void ReplaceHeightmap(Texture2D value) { }
        public void SetHeightmap(Texture2D value) { heightmap=new Texture{width=value.width,height=value.height,Raw=value.Raw.ToArray()}; }
        public void SetWorldHeightmap(Texture2D value,bool copy) { worldHeightmap=new Texture{width=value.width,height=value.height,Raw=value.Raw.ToArray()}; }
        public void FinalizeTerrainData(Texture2D map,Texture2D? world,Unity.Mathematics.float2 scale,Unity.Mathematics.float2 corner,Unity.Mathematics.float2 area,Unity.Mathematics.float2 worldCorner,Unity.Mathematics.float2 worldArea,Unity.Mathematics.float2 range)
        {if(FailFinalizeOnce){FailFinalizeOnce=false;throw new InvalidOperationException("Injected finalization failure");}heightScaleOffset=scale;playableOffset=corner;playableArea=area;}
    }
    public class WaterSystem { public bool Loaded; public float SeaLevel; public bool UseActiveCellsCulling=true; public void TerrainWillChange(){} public void TerrainWillChangeFromBrush(Colossal.Mathematics.Bounds2 bounds)=>throw new NotSupportedException("Native brush notification is not simulated"); }
}
namespace ImportEarthLocation.Core
{
    public class TestMapSize { public double WidthMeters = 57344; }
    public class TestRequest { public TestMapSize MapSize = new(); }
    public class ImportRequest : TestRequest { public long MapSessionId,Generation; }
    public static class ImportSession { public static bool Matches(long map,long generation)=>false; }
    public class PreparedImport { public ushort[] Heightmap = new ushort[16]; public float HeightScaleMeters, SeaLevelMeters; public TestRequest Request = new(); }
    public class ImportLog { public void Diagnostic(string s){} }
}
namespace ImportEarthLocation.Processing { public static class TerrainBuilder { public const int Resolution = 4; } }
namespace ImportEarthLocation { public static class Mod { public const string Id = "ImportEarthLocation"; } }
