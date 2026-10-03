using System;
using System.Collections.Generic;
using System.Diagnostics;
using Colossal.Mathematics;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using ImportEarthLocation.Integration;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Systems;

public sealed class NativeNetworkPlacement
{
    private readonly EntityManager _entityManager;
    private readonly PrefabResolver _prefabs;
    private readonly ImportLog _log;
    public int RoadSections { get; private set; }
    public int RailSections { get; private set; }
    private readonly HashSet<string> _isolatedEndpoints = new(StringComparer.Ordinal);
    // A shared source node at a bridge/tunnel portal is a real connection. Grade-separated
    // crossings with different source node IDs remain isolated, even at coincident positions.
    private static string Identity(PreparedSegment segment, long node) => $"{segment.Kind}:{node}";

    public NativeNetworkPlacement(EntityManager entityManager, PrefabSystem prefabSystem, ImportLog log)
    {
        _entityManager = entityManager;
        _prefabs = new PrefabResolver(prefabSystem, log);
        _log = log;
    }

    public static IReadOnlyList<PreparedSegment> Preflight(IReadOnlyList<PreparedSegment> segments, PrefabSystem prefabs, ImportLog log)
    {
        var resolver=new PrefabResolver(prefabs,log);var supported=new List<PreparedSegment>();var rejected=0;
        foreach(var s in segments)
        {
            if(resolver.Resolve(s)!=Entity.Null){supported.Add(s);continue;}
            rejected++;log.Count(s.Kind+" unsupported native direction/width");
            if(rejected<=25)log.Warn($"OSM {s.Kind} way {s.WayId} omitted before replacement: no verified prefab for direction/lanes/available width ({s.MaximumNativeWidth:0.0} m).");
        }
        foreach(var kind in new[]{NetworkKind.Road,NetworkKind.Rail})
        {
            var count=System.Linq.Enumerable.Count(segments,s=>s.Kind==kind);var kept=System.Linq.Enumerable.Count(supported,s=>s.Kind==kind);
            if(count>0&&kept==0||count>=20&&kept<count/2)throw new InvalidOperationException($"Major {kind} prefab compatibility failure: {count-kept}/{count} sections unsupported. Current map unchanged.");
        }
        if(rejected>0)log.Warn($"Native network preflight omitted {rejected}/{segments.Count} unsupported sections; this is a partial network import, not clean success.");
        return supported;
    }

    public IReadOnlyList<Entity> CreateDefinitions(IReadOnlyList<PreparedSegment> segments, out int skipped)
    {
        var definitions = new List<Entity>(segments.Count);
        var positions = new Dictionary<float3, HashSet<string>>();
        foreach (var segment in segments)
        foreach (var endpoint in new[] { (segment.Start, segment.StartNodeId), (segment.End, segment.EndNodeId) })
        {
            if (!positions.TryGetValue(endpoint.Item1, out var identities)) positions[endpoint.Item1] = identities = new HashSet<string>();
            identities.Add(Identity(segment, endpoint.Item2));
        }
        foreach (var identities in positions.Values) if (identities.Count > 1) foreach (var identity in identities) _isolatedEndpoints.Add(identity);
        skipped = 0;
        var roads = Stopwatch.StartNew(); var roadTime = 0d; var railTime = 0d;
        foreach (var segment in segments)
        {
            roads.Restart();
            try
            {
                var prefab = _prefabs.Resolve(segment);
                if (prefab == Entity.Null) { skipped++; _log.Count(segment.Kind + " skipped"); continue; }
                definitions.Add(CreateDefinition(segment, prefab));
                if (segment.Kind == NetworkKind.Road) RoadSections++; else RailSections++;
                _log.Count(segment.Kind + " definitions queued");
            }
            catch (Exception exception)
            {
                skipped++;
                _log.Count(segment.Kind + " skipped");
                if (skipped <= 25) _log.Warn($"Skipped OSM way {segment.WayId} segment {segment.StartNodeId}-{segment.EndNodeId}: {exception.Message}");
            }
            finally { if (segment.Kind == NetworkKind.Road) roadTime += roads.Elapsed.TotalSeconds; else railTime += roads.Elapsed.TotalSeconds; }
        }
        if (RoadSections > 0) _log.Time("Road apply", roadTime);
        if (RailSections > 0) _log.Time("Rail apply", railTime);
        if (segments.Count > 20 && skipped > segments.Count / 2)
        {
            foreach (var definition in definitions) _entityManager.DestroyEntity(definition);
            throw new InvalidOperationException($"Major network placement failure: {skipped}/{segments.Count} sections failed. The map may have changed.");
        }
        return definitions;
    }

    private Entity CreateDefinition(PreparedSegment segment, Entity prefab)
    {
        var curve = segment.Control1.HasValue && segment.Control2.HasValue
            ? new Bezier4x3(segment.Start, segment.Control1.Value, segment.Control2.Value, segment.End)
            : NetUtils.StraightCurve(segment.Start, segment.End);
        var direction = math.normalizesafe((segment.Control1 ?? segment.End) - segment.Start, new float3(0f, 0f, 1f));
        var rotation = quaternion.LookRotationSafe(direction, new float3(0f, 1f, 0f));
        var endDirection = math.normalizesafe(segment.End - (segment.Control2 ?? segment.Start), direction);
        var elevation = segment.Bridge ? Math.Max(8f, segment.Layer * 6f) : segment.Tunnel ? Math.Min(-8f, segment.Layer * 6f) : segment.Layer * 6f;
        var startElevation = segment.StartElevation ?? elevation; var endElevation = segment.EndElevation ?? elevation;
        var startPosition = CoursePosition(segment.Start, rotation, true, startElevation);
        var endPosition = CoursePosition(segment.End, quaternion.LookRotationSafe(endDirection, new float3(0, 1, 0)), false, endElevation);
        if (_isolatedEndpoints.Contains(Identity(segment, segment.StartNodeId))) startPosition.m_Flags |= CoursePosFlags.DisableMerge;
        if (_isolatedEndpoints.Contains(Identity(segment, segment.EndNodeId))) endPosition.m_Flags |= CoursePosFlags.DisableMerge;
        var entity = _entityManager.CreateEntity();
        try
        {
        _entityManager.AddComponentData(entity, new CreationDefinition
        {
            m_Prefab = prefab,
            m_SubPrefab = Entity.Null,
            m_Original = Entity.Null,
            m_Owner = Entity.Null,
            m_Attached = Entity.Null,
            m_Flags = CreationFlags.SubElevation,
            m_RandomSeed = (int)(math.hash(segment.Start) ^ math.hash(segment.End))
        });
        _entityManager.AddComponentData(entity, new NetCourse
        {
            m_Curve = curve,
            m_StartPosition = startPosition,
            m_EndPosition = endPosition,
            m_Elevation = new float2(startElevation, endElevation),
            m_Length = MathUtils.Length(curve),
            m_FixedIndex = -1
        });
        _entityManager.AddComponentData(entity, default(Updated));
        return entity;
        }
        catch { _entityManager.DestroyEntity(entity); throw; }
    }

    private static CoursePos CoursePosition(float3 position, quaternion rotation, bool start, float elevation) => new()
    {
        m_Entity = Entity.Null,
        m_Position = position,
        m_Rotation = rotation,
        m_Elevation = new float2(elevation),
        m_CourseDelta = start ? 0f : 1f,
        m_SplitPosition = 0f,
        m_Flags = (start ? CoursePosFlags.IsFirst : CoursePosFlags.IsLast) | CoursePosFlags.IsLeft | CoursePosFlags.IsRight |
            (elevation != 0 ? CoursePosFlags.FreeHeight | CoursePosFlags.ForceElevatedNode | CoursePosFlags.ForceElevatedEdge : 0),
        m_ParentMesh = -1
    };
}
