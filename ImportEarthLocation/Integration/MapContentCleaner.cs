using Game.Common;
using Game.Net;
using Game.Simulation;
using Game.Tools;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

public static class MapContentCleaner
{
    public static (int networks, int waterSources) MarkForDeletion(EntityManager entityManager)
    {
        entityManager.CompleteAllTrackedJobs();
        using var networkQuery = entityManager.CreateEntityQuery(new EntityQueryDesc
        {
            Any = new[] { ComponentType.ReadOnly<Edge>(), ComponentType.ReadOnly<Node>() },
            None = new[] { ComponentType.ReadOnly<Deleted>(), ComponentType.ReadOnly<Temp>() }
        });
        using var waterQuery = entityManager.CreateEntityQuery(new EntityQueryDesc
        {
            All = new[] { ComponentType.ReadOnly<WaterSourceData>() },
            None = new[] { ComponentType.ReadOnly<Deleted>() }
        });
        var networks = networkQuery.CalculateEntityCount();
        var water = waterQuery.CalculateEntityCount();
        if (networks > 0) entityManager.AddComponent<Deleted>(networkQuery);
        if (water > 0) entityManager.AddComponent<Deleted>(waterQuery);
        return (networks, water);
    }
}
