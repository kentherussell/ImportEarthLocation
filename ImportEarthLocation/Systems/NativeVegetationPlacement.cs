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

public sealed class NativeVegetationPlacement
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
    {
        if(start<0||count<0||start>points.Count-count)throw new ArgumentOutOfRangeException(nameof(start));
        var slice=new VegetationCandidate[count];for(var i=0;i<count;i++){var p=points[start+i];slice[i]=new(p,VegetationForm.Broadleaf,0,math.hash(p.xz),0);}
        return CreateBatch(manager,slice,0,count,
            prefabs.Select(p=>new StockPrefab(manager.World.GetExistingSystemManaged<PrefabSystem>()?.GetPrefabName(p) ?? "",p,VegetationForm.Broadleaf)).ToArray(),log);
    }

    private readonly EntityManager _manager;
    private readonly ImportLog _log;
    private readonly ImportEpoch _epoch;
    private readonly StockPrefab[] _catalog;
    private readonly int[][] _palettes=new int[4][];
    private readonly bool[] _valid;
    private readonly int[] _assetCounts;
    private readonly int[] _formCounts=new int[4];
    private readonly EntityArchetype _archetype;
    private readonly Func<float3,bool>? _localRoadExclusion;
    private readonly List<Entity> _definitions=new(8192);
    private readonly IReadOnlyList<Entity> _view;
    public NativeVegetationPlacement(EntityManager manager,StockPrefab[] catalog,ImportLog log,Func<float3,bool>? localRoadExclusion=null)
    {
        _manager=manager;_log=log;_epoch=new ImportEpoch(ImportSession.Map,ImportSession.Generation,manager.World.SequenceNumber);
        _localRoadExclusion=localRoadExclusion;
        _catalog=(StockPrefab[])catalog.Clone();_valid=new bool[catalog.Length];_assetCounts=new int[catalog.Length];_view=_definitions.AsReadOnly();
        for(var form=0;form<4;form++)
        {
            var indices=new List<int>();
            for(var i=0;i<_catalog.Length;i++)if((int)_catalog[i].Form==form&&!Placeholder(_catalog[i].Name))indices.Add(i);
            _palettes[form]=indices.ToArray();
        }
        _archetype=manager.CreateArchetype(ComponentType.ReadWrite<CreationDefinition>(),ComponentType.ReadWrite<ObjectDefinition>(),ComponentType.ReadWrite<Updated>());
    }

    // Compatibility adapter; production retains one manager/epoch-bound instance.
    public static IReadOnlyList<Entity> CreateBatch(EntityManager manager,IReadOnlyList<VegetationCandidate> points,int start,int count,
        StockPrefab[] prefabs,ImportLog log) => new NativeVegetationPlacement(manager,prefabs,log).CreateBatch(points,start,count);

    public IReadOnlyList<Entity> CreateBatch(IReadOnlyList<VegetationCandidate> points,int start,int count)
    {
        if(start<0||count<0||start>points.Count-count)throw new ArgumentOutOfRangeException(nameof(start));
        if(!_manager.World.IsCreated||!ImportSession.Matches(_epoch,_manager.World.SequenceNumber))
            throw new InvalidOperationException("Vegetation catalog belongs to a retired world/import.");
        NetworkCommitSystem.BeginDefinitionBatch(_manager,_log);
        _definitions.Clear();Array.Clear(_formCounts,0,4);Array.Clear(_assetCounts,0,_assetCounts.Length);
        for(var i=0;i<_catalog.Length;i++)
        {
            var entity=_catalog[i].Entity;
            _valid[i]=_manager.Exists(entity)&&_manager.HasComponent<PlantData>(entity)&&_manager.HasComponent<PrefabData>(entity);
        }
        var failures=0;var nonfinite=0;var retired=0;var missing=new int[4];
        using var allocated=new NativeArray<Entity>(count,Allocator.Temp);
        try
        {
            _manager.CreateEntity(_archetype,allocated);
            foreach(var entity in allocated)NetworkCommitSystem.RegisterDefinition(entity);
        }
        catch { NetworkCommitSystem.AllocationUncertain();throw; }
        for(var offset=0;offset<count;offset++)
        {
            var entity=allocated[offset];
            try
            {
                var candidate=points[start+offset];var point=candidate.Position;var seed=candidate.Seed;
                if(!math.all(math.isfinite(point))){_manager.DestroyEntity(entity);nonfinite++;continue;}
                if(_localRoadExclusion?.Invoke(point)==true)
                {_manager.DestroyEntity(entity);_log.Count("Vegetation excluded by revised road footprint");continue;}
                var form=(int)candidate.Form;
                if(form<0||form>=4){_manager.DestroyEntity(entity);failures++;continue;}
                var palette=_palettes[form];
                if(palette.Length==0){_manager.DestroyEntity(entity);missing[form]++;continue;}
                var index=palette[seed%(uint)palette.Length];var selected=_catalog[index];
                if(!_valid[index]){_manager.DestroyEntity(entity);retired++;continue;}
                _manager.SetComponentData(entity,new CreationDefinition
                {
                    m_Prefab=selected.Entity,m_SubPrefab=Entity.Null,m_Original=Entity.Null,
                    m_Owner=Entity.Null,m_Attached=Entity.Null,m_Flags=0,m_RandomSeed=(int)seed
                });
                _manager.SetComponentData(entity,new ObjectDefinition
                {
                    m_Position=point,m_LocalPosition=float3.zero,m_Scale=new float3(1),
                    m_Rotation=quaternion.RotateY(seed/(float)uint.MaxValue*math.PI*2),m_LocalRotation=quaternion.identity,
                    m_Elevation=0,m_Intensity=1,m_Age=1,m_ParentMesh=-1,m_GroupIndex=-1,m_Probability=100,m_PrefabSubIndex=-1
                });
                _definitions.Add(entity);_formCounts[form]++;_assetCounts[index]++;
            }
            catch(Exception error) when(error is not OutOfMemoryException&&error is not OperationCanceledException)
            {
                if(entity!=Entity.Null)_manager.DestroyEntity(entity);
                if(++failures<=5)_log.Warn($"Skipped tree candidate {start+offset}: {error.Message}");
            }
        }
        // Merge bounded primitive counters once; no per-site metric strings/locks.
        for(var form=0;form<4;form++)
        {
            if(_formCounts[form]>0)_log.Count("Vegetation definitions "+(VegetationForm)form,_formCounts[form]);
            if(missing[form]>0)_log.Count("Vegetation unavailable stock palette "+(VegetationForm)form,missing[form]);
        }
        for(var i=0;i<_catalog.Length;i++)if(_assetCounts[i]>0)_log.Count("Vegetation definitions asset "+_catalog[i].Name,_assetCounts[i]);
        if(nonfinite>0)_log.Count("Vegetation invalid coordinates omitted",nonfinite);
        if(retired>0)_log.Count("Vegetation retired prefabs omitted",retired);
        if(failures>0){_log.Count("Trees skipped",failures);_log.Count("Vegetation definition failures",failures);}
        NetworkCommitSystem.DescribeFoliageDefinitions(count,_definitions.Count);
        if(_log.DiagnosticsEnabled)_log.Diagnostic($"VEGETATION ALLOCATION requested={count}; definitions={_definitions.Count}; bulkArchetypeAllocation=true; nativeInitializationPreserved=true; catalogCached=true");
        if(_definitions.Count==0)NetworkCommitSystem.ReleaseEmptyDefinitions(_manager);
        return _view; // Remains unchanged until settlement and the next CreateBatch.
    }
}
