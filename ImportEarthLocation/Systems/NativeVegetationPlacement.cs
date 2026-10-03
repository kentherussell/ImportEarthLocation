using System;
using System.Collections.Generic;
using System.Linq;
using Game.Common;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Systems;

public static class NativeVegetationPlacement
{
    public static Entity[] ResolvePrefabs(EntityManager manager, PrefabSystem prefabs)
    {
        using var query = manager.CreateEntityQuery(ComponentType.ReadOnly<TreeData>(), ComponentType.ReadOnly<PrefabData>());
        using var entities = query.ToEntityArray(Allocator.Temp);
        var available = new List<(string name, Entity entity)>();
        foreach (var entity in entities) available.Add((prefabs.GetPrefabName(entity), entity));
        return available.OrderBy(item => item.name, StringComparer.Ordinal).Select(item => item.entity).ToArray();
    }

    public static IReadOnlyList<Entity> CreateBatch(EntityManager manager, IReadOnlyList<float3> points, int start, int count,
        Entity[] prefabs, ImportLog log)
    {
        var definitions = new List<Entity>(count);
        var failures = 0;
        for (var i = start; i < start + count; i++)
        {
            var entity = Entity.Null;
            try
            {
                var point = points[i]; var seed = math.hash(point);
                entity = manager.CreateEntity();
                manager.AddComponentData(entity, new CreationDefinition
                {
                    m_Prefab = prefabs[seed % (uint)prefabs.Length], m_SubPrefab = Entity.Null, m_Original = Entity.Null,
                    m_Owner = Entity.Null, m_Attached = Entity.Null, m_Flags = 0, m_RandomSeed = (int)seed
                });
                manager.AddComponentData(entity, new ObjectDefinition
                {
                    m_Position = point, m_LocalPosition = float3.zero, m_Scale = new float3(1),
                    m_Rotation = quaternion.RotateY(seed / (float)uint.MaxValue * math.PI * 2), m_LocalRotation = quaternion.identity,
                    m_Elevation = 0, m_Intensity = 1, m_Age = 1, m_ParentMesh = -1, m_GroupIndex = -1, m_Probability = 100, m_PrefabSubIndex = -1
                });
                manager.AddComponentData(entity, default(Updated)); definitions.Add(entity);
            }
            catch (Exception exception)
            {
                if (entity != Entity.Null) manager.DestroyEntity(entity);
                log.Count("Trees skipped");
                if (++failures <= 5) log.Warn($"Skipped tree candidate {i}: {exception.Message}");
            }
        }
        if (failures > 5) log.Warn($"Tree batch skipped {failures} candidates; additional individual warnings suppressed.");
        return definitions;
    }
}
