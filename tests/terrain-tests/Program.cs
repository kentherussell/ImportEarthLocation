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
        bridge.Apply(new ImportEarthLocation.Core.PreparedImport { SeaLevelMeters = 40 });
        Check(water.SeaLevel == 41, "Applied water was not exactly 1m above nominal sea level");
        Console.WriteLine("PASS: ready terrain accepted without transient flag; missing render/CPU data, fallback resolution and unloaded water rejected; reload accepted");
    }
}
namespace Unity.Mathematics
{
    public struct float2 { public float x, y; public float2(float x, float y) { this.x = x; this.y = y; } }
    public struct int3 { public int x, y, z; }
}
namespace UnityEngine
{
    public class Object { public static void Destroy(Object value) { } }
    public class Texture : Object { public int width, height; }
    public enum TextureFormat { R16 }
    public class Texture2D : Texture
    {
        public string name = "";
        public Texture2D(int width, int height, TextureFormat format, bool mipChain, bool linear) { this.width = width; this.height = height; }
        public void LoadRawTextureData(byte[] bytes) { }
        public void Apply(bool updateMipmaps, bool makeNoLongerReadable) { }
    }
}
namespace Game.Simulation
{
    public struct HeightArray { public bool IsCreated; public int Length; }
    public struct TerrainHeightData { public HeightArray heights; public Unity.Mathematics.int3 resolution; }
    public class TerrainSystem
    {
        public Texture? heightmap = new Texture { width = 4096, height = 4096 };
        public TerrainHeightData Data = new() { heights = new() { IsCreated = true, Length = 16 }, resolution = new() { x = 4, z = 4 } };
        public TerrainHeightData GetHeightData(bool sync) => Data;
        public void SetTerrainProperties(Unity.Mathematics.float2 value) { }
        public void ReplaceHeightmap(Texture2D value) { }
    }
    public class WaterSystem { public bool Loaded; public float SeaLevel; }
}
namespace ImportEarthLocation.Core
{
    public class PreparedImport { public ushort[] Heightmap = new ushort[16]; public float HeightScaleMeters, SeaLevelMeters; }
}
namespace ImportEarthLocation.Processing { public static class TerrainBuilder { public const int Resolution = 4; } }
namespace ImportEarthLocation { public static class Mod { public const string Id = "ImportEarthLocation"; } }
