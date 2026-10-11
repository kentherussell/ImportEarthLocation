using System.Collections.Generic;
using System.Linq;
using Game.Prefabs;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

public static class NativePrefabCatalog
{
    public static T[] Read<T>(EntityManager manager, PrefabSystem prefabs) where T : PrefabBase
    {
        using var query = manager.CreateEntityQuery(ComponentType.ReadOnly<PrefabData>());
        using var entities = query.ToEntityArray(Allocator.Temp);
        var result = new List<T>();
        foreach (var entity in entities) if (prefabs.TryGetPrefab<T>(entity, out var prefab) && prefab != null) result.Add(prefab);
        return result.OrderBy(p => p.name, System.StringComparer.Ordinal).ToArray();
    }
}
