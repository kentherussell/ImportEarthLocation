// Offline stand-ins for the native API; these tests execute the production
// resolver, not the game. Installed-game metadata/build checks cover API shape.
namespace Unity.Entities
{
    public record struct Entity(int Id) { public static Entity Null => default; }
    public class EntityManager
    {
        public readonly Dictionary<Entity, object> Components = new();
        public readonly Dictionary<Entity, List<Game.Prefabs.DefaultNetLane>> Lanes = new();
        public readonly Dictionary<(Entity,Type),object> TypedComponents = new();
        public bool HasComponent<T>(Entity e) => TypedComponents.ContainsKey((e,typeof(T))) || Components.TryGetValue(e, out var c) && c is T;
        public T GetComponentData<T>(Entity e) => TypedComponents.TryGetValue((e,typeof(T)),out var c)?(T)c:(T)Components[e];
        public bool HasBuffer<T>(Entity e) => Lanes.ContainsKey(e);
        public List<Game.Prefabs.DefaultNetLane> GetBuffer<T>(Entity e, bool readOnly) => Lanes[e];
    }
}
namespace Game.Net { [Flags] public enum TrackTypes { Train = 1, Tram = 2 } }
namespace Game.Prefabs
{
    using Unity.Entities;
    public class PrefabBase
    {
        public string name = "";
        public Entity Entity;
        public readonly Dictionary<Type, object> Components = new();
        public bool TryGet<T>(out T value) { value = Components.TryGetValue(typeof(T), out var c) ? (T)c : default!; return c is T; }
    }
    public class RoadPrefab : PrefabBase { public bool m_HighwayRules; public NetSectionInfo[]? m_Sections; }
    public class TrackPrefab : PrefabBase { public Game.Net.TrackTypes m_TrackType; public NetSectionInfo[]? m_Sections; }
    public class NetSectionPrefab : PrefabBase { public NetPieceInfo[]? m_Pieces; public NetSubSectionInfo[]? m_SubSections; }
    public class NetPiecePrefab : PrefabBase { }
    public class NetLanePrefab : PrefabBase { }
    public class NetPieceLanes { public NetLaneInfo[]? m_Lanes; }
    public class TrackLane { public Game.Net.TrackTypes m_TrackType; }
    public class CarLane { }
    public struct CarLaneData { }
    public struct Position { public float x; }
    public struct NetSectionInfo { public NetSectionPrefab m_Section; public Position m_Offset; public bool m_Invert; }
    public struct NetSubSectionInfo { public NetSectionPrefab m_Section; }
    public struct NetPieceInfo { public NetPiecePrefab m_Piece; public Position m_Offset; }
    public struct NetLaneInfo { public NetLanePrefab m_Lane; public Position m_Position; }
    public struct DefaultNetLane { public Entity m_Lane; public Position m_Position; }
    public struct TrackLaneData { public Game.Net.TrackTypes m_TrackTypes; }
    public struct NetGeometryData { public float m_DefaultWidth, m_ElevatedWidth; }
    [Flags] public enum RoadFlags { DefaultIsForward = 1, DefaultIsBackward = 2 }
    public struct RoadData { public RoadFlags m_Flags; }
    public record struct PrefabID(string Type, string Name);
    public class PrefabSystem
    {
        public EntityManager EntityManager = new();
        public List<PrefabBase> Catalog = new();
        public Entity GetEntity(PrefabBase p) => p.Entity;
        public T GetPrefab<T>(Entity entity) where T:PrefabBase => Catalog.OfType<T>().Single(p=>p.Entity==entity);
        public bool TryGetPrefab(PrefabID id, out PrefabBase p) { p = Catalog.FirstOrDefault(x => x.name == id.Name)!; return p != null; }
    }
}
namespace ImportEarthLocation.Integration
{
    public static class NativePrefabCatalog
    { public static IEnumerable<T> Read<T>(Unity.Entities.EntityManager manager, Game.Prefabs.PrefabSystem system) => system.Catalog.OfType<T>(); }
}
namespace ImportEarthLocation.Core
{
    public enum NetworkKind { Road, Rail }
    public record PreparedSegment(long WayId, long StartNodeId, long EndNodeId, object? Start, object? End, NetworkKind Kind, string Classification, int Lanes, bool OneWay, bool Bridge, bool Tunnel, int Layer)
    { public bool ConsolidatedAvenue; public int PhysicalTracks; public float MaximumNativeWidth = float.PositiveInfinity; public float NativeWidth; public IReadOnlyList<float> NativeLaneOffsets = Array.Empty<float>(); }
    public class ImportLog
    {
        public readonly List<string> Messages = new();
        public void Warn(string s) => Messages.Add(s);
        public void Diagnostic(string s) => Messages.Add(s);
        public void Count(string s) => Messages.Add("COUNT "+s);
    }
    public static class Numeric { public static int Clamp(int n, int min, int max) => Math.Clamp(n, min, max); }
}
