using System;
using System.Collections.Generic;
using System.Linq;
using Game.Common;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Colossal.IO.AssetDatabase;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Systems;

public static class NativeVegetationPlacement
{
    public sealed record StockPrefab(string Name,Entity Entity,VegetationForm Form);
    public static Entity[] ResolvePrefabs(EntityManager manager, PrefabSystem prefabs)
        => ResolveCatalog(manager,prefabs).Select(p=>p.Entity).ToArray();

    public static StockPrefab[] ResolveCatalog(EntityManager manager,PrefabSystem prefabs,ImportLog? log=null)
    {
        using var query = manager.CreateEntityQuery(ComponentType.ReadOnly<PlantData>(), ComponentType.ReadOnly<PrefabData>());
        using var entities = query.ToEntityArray(Allocator.Temp);
        var available = new List<StockPrefab>();
        foreach (var entity in entities)
        {
            if(!prefabs.TryGetPrefab<PrefabBase>(entity,out var prefab)||prefab==null||!prefab.isBuiltin||prefab.isSubscribedMod)continue;
            // Serialized assets must come from the base-game database. Built-in
            // Unity prefabs are verified by PrefabBase's native GUID map instead.
            if(prefab.asset!=null&&prefab.asset.database!=AssetDatabase.game)continue;
            if(RequiresPaidContent(prefab)||(prefab.TryGet<ContentPrerequisite>(out var content)&&content.m_ContentPrerequisite!=null&&RequiresPaidContent(content.m_ContentPrerequisite)))
            {log?.Count("Vegetation paid-content assets excluded");continue;}
            var name=prefabs.GetPrefabName(entity);var lower=name.ToLowerInvariant();
            if (Placeholder(name))
            { log?.Count("Vegetation placeholder assets excluded"); log?.Diagnostic($"VEGETATION EXCLUDED name={name}; reason=placeholder-not-renderable-foliage; fallbackAllowed=false"); continue; }
            var tree=manager.HasComponent<TreeData>(entity);
            if(!tree&&!lower.Contains("bush")&&!lower.Contains("shrub")&&!lower.Contains("hedge"))continue;
            var form=!tree||lower.Contains("bush")||lower.Contains("shrub")||lower.Contains("hedge")?VegetationForm.Shrub:lower.Contains("palm")?VegetationForm.Palm:
                lower.Contains("pine")||lower.Contains("spruce")||lower.Contains("fir")||lower.Contains("cedar")||lower.Contains("cypress")?VegetationForm.Conifer:VegetationForm.Broadleaf;
            available.Add(new(name,entity,form));
            log?.Diagnostic($"VEGETATION STOCK name={name}; form={form}; builtin={prefab.isBuiltin}; origin={(prefab.asset==null?"native-guid-map":"game-database")}; asset={prefab.asset?.id.ToString()??"native"}; subscribedMod=false");
        }
        log?.Diagnostic($"VEGETATION CATALOG policy=base-game-only; assets={available.Count}; trees={available.Count(p=>p.Form!=VegetationForm.Shrub)}; bushes={available.Count(p=>p.Form==VegetationForm.Shrub)}; paidPackFallback=false; modifiedBuiltinAppearanceNotVerifiable=true");
        return available.OrderBy(item => item.Name, StringComparer.Ordinal).ToArray();
    }

    private static bool RequiresPaidContent(PrefabBase prefab)
    {
        // Keep lists independent: native TryGet(list) may clear the supplied
        // list, so a content prerequisite cannot hide a direct DLC requirement.
        var requirements=new List<DlcRequirement>();prefab.TryGet(requirements);
        return requirements.Any(r=>!r.m_Dlc.Equals(default(Colossal.PSI.Common.DlcId)));
    }

    private static bool Placeholder(string name) =>
        string.IsNullOrWhiteSpace(name) || name.IndexOf("placeholder", StringComparison.OrdinalIgnoreCase) >= 0;

    public static IReadOnlyList<Entity> CreateBatch(EntityManager manager, IReadOnlyList<float3> points, int start, int count,
        Entity[] prefabs, ImportLog log)
        => CreateBatch(manager,points.Select(p=>new VegetationCandidate(p,VegetationForm.Broadleaf,math.hash(p.xz),0,"legacy","")).ToArray(),start,count,
            prefabs.Select(p=>new StockPrefab(manager.World.GetExistingSystemManaged<PrefabSystem>()?.GetPrefabName(p) ?? "",p,VegetationForm.Broadleaf)).ToArray(),log);

    public static IReadOnlyList<Entity> CreateBatch(EntityManager manager,IReadOnlyList<VegetationCandidate> points,int start,int count,
        StockPrefab[] prefabs,ImportLog log)
    {
        var definitions = new List<Entity>(count);
        var failures = 0;
        var archetype=manager.CreateArchetype(ComponentType.ReadWrite<CreationDefinition>(),ComponentType.ReadWrite<ObjectDefinition>(),ComponentType.ReadWrite<Updated>());
        var counts=new Dictionary<string,int>();
        void Count(string name){counts.TryGetValue(name,out var value);counts[name]=value+1;}
        var palettes=Enum.GetValues(typeof(VegetationForm)).Cast<VegetationForm>().ToDictionary(form=>form,form=>prefabs.Where(p=>p.Form==form && !Placeholder(p.Name)).ToArray());
        using var allocated=new NativeArray<Entity>(count,Allocator.Temp);
        manager.CreateEntity(archetype,allocated);
        for (var i = start; i < start + count; i++)
        {
            var entity = allocated[i-start];
            try
            {
                var candidate=points[i];var point=candidate.Position;var seed=candidate.Seed;
                var palette=palettes[candidate.Form];
                // Unknown exact species may use a same-form stock asset, but a
                // missing palm/conifer palette must not become unrelated foliage.
                if(palette.Length==0){manager.DestroyEntity(entity);Count("Vegetation unavailable stock palette "+candidate.Form);continue;}
                var selected=palette[seed%(uint)palette.Length];
                // Allocate the final definition archetype once, avoiding three
                // structural migrations per candidate. Native initialization is
                // unchanged: these remain tool definitions, not raw Tree entities.
                manager.SetComponentData(entity, new CreationDefinition
                {
                    m_Prefab = selected.Entity, m_SubPrefab = Entity.Null, m_Original = Entity.Null,
                    m_Owner = Entity.Null, m_Attached = Entity.Null, m_Flags = 0, m_RandomSeed = (int)seed
                });
                manager.SetComponentData(entity, new ObjectDefinition
                {
                    m_Position = point, m_LocalPosition = float3.zero, m_Scale = new float3(1),
                    m_Rotation = quaternion.RotateY(seed / (float)uint.MaxValue * math.PI * 2), m_LocalRotation = quaternion.identity,
                    m_Elevation = 0, m_Intensity = 1, m_Age = 1, m_ParentMesh = -1, m_GroupIndex = -1, m_Probability = 100, m_PrefabSubIndex = -1
                });
                definitions.Add(entity);
                Count("Vegetation definitions "+selected.Form);
                Count("Vegetation definitions asset "+selected.Name);
            }
            catch (Exception exception) when(exception is not OutOfMemoryException&&exception is not OperationCanceledException)
            {
                if (entity != Entity.Null) manager.DestroyEntity(entity);
                log.Count("Trees skipped");
                log.Count("Vegetation definition failures");
                if (++failures <= 5) log.Warn($"Skipped tree candidate {i}: {exception.Message}");
            }
        }
        foreach(var pair in counts)log.Count(pair.Key,pair.Value);
        log.Diagnostic($"VEGETATION ALLOCATION requested={count}; definitions={definitions.Count}; bulkArchetypeAllocation=true; nativeInitializationPreserved=true");
        if (failures > 5) log.Warn($"Tree batch skipped {failures} candidates; additional individual warnings suppressed.");
        return definitions;
    }
}
