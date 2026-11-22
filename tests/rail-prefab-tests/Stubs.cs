// Offline stand-ins for the native API; these tests execute the production
// resolver, not the game. Installed-game metadata/build checks cover API shape.
namespace Unity.Entities
{
    public readonly record struct ComponentType(Type Type)
    {
        public static ComponentType ReadWrite<T>() => new(typeof(T));
    }

    public sealed class ComponentTypes : List<ComponentType>, IDisposable
    {
        public void Dispose() { }
    }

    public sealed class EntityArchetype
    {
        public ComponentTypes Types = new();

        public ComponentTypes GetComponentTypes(Unity.Collections.Allocator allocator) => Types;
    }

    public record struct Entity(int Id)
    {
        public static Entity Null => default;
    }

    public class EntityManager
    {
        public readonly Dictionary<Entity, object> Components = new();
        public readonly Dictionary<Entity, List<Game.Prefabs.DefaultNetLane>> Lanes = new();
        public readonly Dictionary<(Entity, Type), object> TypedComponents = new();

        public bool HasComponent<T>(Entity entity)
            => TypedComponents.ContainsKey((entity, typeof(T))) || Components.TryGetValue(entity, out var c) && c is T;

        public T GetComponentData<T>(Entity entity)
            => TypedComponents.TryGetValue((entity, typeof(T)), out var c) ? (T)c : (T)Components[entity];

        public bool HasBuffer<T>(Entity entity) => Lanes.ContainsKey(entity);

        public List<Game.Prefabs.DefaultNetLane> GetBuffer<T>(Entity entity, bool readOnly) => Lanes[entity];
    }
}
namespace Unity.Collections
{
    public enum Allocator
    {
        Temp
    }
}
namespace Game.Objects
{
    public struct Attached
    {
    }

    public struct Transform
    {
    }
}
namespace Game.Net
{
    [Flags]
    public enum TrackTypes
    {
        Train = 1,
        Tram = 2
    }
}
namespace Game.Prefabs
{
    using Unity.Entities;
    public struct CompositionFlags
    {
        public enum General
        {
            Roundabout = 1
        }

        public General m_General;
    }

    public class PrefabBase
    {
        public string name = "";
        public Entity Entity;
        public readonly Dictionary<Type, object> Components = new();

        public bool TryGet<T>(out T value)
        {
            value = Components.TryGetValue(typeof(T), out var c) ? (T)c : default!;
            return c is T;
        }
    }

    public class RoadPrefab : PrefabBase
    {
        public bool m_HighwayRules;
        public NetSectionInfo[]? m_Sections;
    }

    public class ObjectPrefab : PrefabBase
    {
    }

    public class TrackPrefab : PrefabBase
    {
        public Game.Net.TrackTypes m_TrackType;
        public NetSectionInfo[]? m_Sections;
    }

    public class NetSectionPrefab : PrefabBase
    {
        public NetPieceInfo[]? m_Pieces;
        public NetSubSectionInfo[]? m_SubSections;
    }

    public class NetPiecePrefab : PrefabBase
    {
    }

    public class NetLanePrefab : PrefabBase
    {
    }

    public class NetPieceLanes
    {
        public NetLaneInfo[]? m_Lanes;
    }

    public class TrackLane
    {
        public Game.Net.TrackTypes m_TrackType;
    }

    public class CarLane
    {
    }

    public struct CarLaneData
    {
    }

    public struct Position
    {
        public float x, z;
    }

    public struct NetSectionInfo
    {
        public NetSectionPrefab m_Section;
        public Position m_Offset;
        public bool m_Invert;
    }

    public struct NetSubSectionInfo
    {
        public NetSectionPrefab m_Section;
    }

    public struct NetPieceInfo
    {
        public NetPiecePrefab m_Piece;
        public Position m_Offset;
    }

    public struct NetLaneInfo
    {
        public NetLanePrefab m_Lane;
        public Position m_Position;
    }

    public struct DefaultNetLane
    {
        public Entity m_Lane;
        public Position m_Position;
    }

    public struct TrackLaneData
    {
        public Game.Net.TrackTypes m_TrackTypes;
    }

    public struct NetGeometryData
    {
        public float m_DefaultWidth, m_ElevatedWidth;
    }

    [Flags]
    public enum RoadFlags
    {
        DefaultIsForward = 1,
        DefaultIsBackward = 2
    }

    public struct RoadData
    {
        public RoadFlags m_Flags;
    }

    public struct PrefabRef
    {
    }

    public struct NetObjectData
    {
        public CompositionFlags m_CompositionFlags;
    }

    public struct ObjectData
    {
        public EntityArchetype m_Archetype;
    }

    public struct ObjectGeometryData
    {
        public Position m_Size;
    }

    public record struct PrefabID(string Type, string Name);

    public class PrefabSystem
    {
        public EntityManager EntityManager = new();
        public List<PrefabBase> Catalog = new();

        public Entity GetEntity(PrefabBase otherBase) => otherBase.Entity;

        public T GetPrefab<T>(Entity entity) where T : PrefabBase
            => Catalog.OfType<T>().Single(p => p.Entity == entity);

        public bool TryGetPrefab(PrefabID id, out PrefabBase otherBase)
        {
            otherBase = Catalog.FirstOrDefault(secondBase => secondBase.name == id.Name)!;
            return otherBase != null;
        }
    }
}
namespace ImportEarthLocation.Integration
{
    public static class NativePrefabCatalog
    {
        public static IEnumerable<T> Read<T>(Unity.Entities.EntityManager manager, Game.Prefabs.PrefabSystem system)
            => system.Catalog.OfType<T>();
    }
}
namespace ImportEarthLocation.Core
{
    public enum NetworkKind
    {
        Road,
        Rail
    }

    public record PreparedSegment(
        long WayId,
        long StartNodeId,
        long EndNodeId,
        object? Start,
        object? End,
        NetworkKind Kind,
        string Classification,
        int Lanes,
        bool OneWay,
        bool Bridge,
        bool Tunnel,
        int Layer)
    {
        public bool ConsolidatedAvenue;
        public int PhysicalTracks;
        public float MaximumNativeWidth = float.PositiveInfinity;
        public float NativeWidth;
        public string NativeAssetKey = "";
        public IReadOnlyList<float> NativeLaneOffsets = Array.Empty<float>();
    }

    public class ImportLog
    {
        public bool StrictRecovery { get; init; }

        public readonly List<string> Messages = new();

        public void Warn(string text) => Messages.Add(text);

        public void Diagnostic(string text) => Messages.Add(text);

        public void Evidence(string text, bool warning = false) => Messages.Add(text);

        public void Count(string text) => Messages.Add("COUNT " + text);
    }

    public static class Numeric
    {
        public static int Clamp(int n, int min, int max) => Math.Clamp(n, min, max);
    }
}
// This harness exercises asset resolution, not roundabout geometry. Actual
// roundabout API shape is verified by the separate installed-reference build.
namespace ImportEarthLocation.Processing
{
    public static class RoundaboutPlanner
    {
        public static void ConfigureStockRadii(IEnumerable<float> radii) { }
    }
}
