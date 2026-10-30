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
    // Sink is metres below the terrain sample: rocks are set into the ground by a
    // fifth of their height so slopes do not show a floating underside. Radius is
    // half the rock's footprint, for the overlap check; 0 for plants.
    public sealed record StockPrefab(string Name,Entity Entity,VegetationForm Form,float Sink=0,float Radius=0);
    private const int FormCount=7;
    private static bool IsRock(VegetationForm form)=>form>=VegetationForm.RockSmall;
    public static Entity[] ResolvePrefabs(EntityManager manager, PrefabSystem prefabs)
        => ResolveCatalog(manager,prefabs).Select(p=>p.Entity).ToArray();

    private const string ShrubAsset="GreenBushWild01";

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
        // Bushes are one defined asset: the random groups mixed city, flowering
        // and hedge forms into wild ground. Others remain only if it is missing.
        if(available.Any(p=>p.Name==ShrubAsset))available.RemoveAll(p=>p.Form==VegetationForm.Shrub&&p.Name!=ShrubAsset);
        else log?.Warn($"Bush asset {ShrubAsset} is unavailable; other base-game bushes are used.");
        log?.Diagnostic($"VEGETATION CATALOG policy=base-game-only; assets={available.Count}; trees={available.Count(p=>p.Form!=VegetationForm.Shrub)}; bushes={available.Count(p=>p.Form==VegetationForm.Shrub)}; paidPackFallback=false; modifiedBuiltinAppearanceNotVerifiable=true");
        available.AddRange(ResolveRocks(manager,prefabs,log));
        return available.OrderBy(item => item.Name, StringComparer.Ordinal).ToArray();
    }

    // Vanilla Rock01–Rock30 and Boulder01–Boulder08 by exact name (the *Random01
    // groups are excluded; seeds already choose among variants). Boulders are the
    // big pieces and anchor clusters (RockLarge); rocks are small debris, the
    // largest third of them RockMedium and the rest RockSmall, by real footprint.
    private static readonly System.Text.RegularExpressions.Regex RockName=new("^(Rock|Boulder)[0-9]+$");
    private static IEnumerable<StockPrefab> ResolveRocks(EntityManager manager,PrefabSystem prefabs,ImportLog? log)
    {
        using var query=manager.CreateEntityQuery(ComponentType.ReadOnly<ObjectGeometryData>(),ComponentType.ReadOnly<ObjectData>(),ComponentType.ReadOnly<PrefabData>());
        using var entities=query.ToEntityArray(Allocator.Temp);
        var found=new List<(string Name,Entity Entity,float Footprint,float Sink,float3 Size)>();
        foreach(var entity in entities)
        {
            if(manager.HasComponent<PlantData>(entity))continue;
            var name=prefabs.GetPrefabName(entity);if(!RockName.IsMatch(name))continue;
            if(!prefabs.TryGetPrefab<PrefabBase>(entity,out var prefab)||prefab==null||!prefab.isBuiltin||prefab.isSubscribedMod)continue;
            if(prefab.asset!=null&&prefab.asset.database!=AssetDatabase.game)continue;
            if(RequiresPaidContent(prefab)||(prefab.TryGet<ContentPrerequisite>(out var content)&&content.m_ContentPrerequisite!=null&&RequiresPaidContent(content.m_ContentPrerequisite)))
            {log?.Count("Rock paid-content assets excluded");continue;}
            var bounds=manager.GetComponentData<ObjectGeometryData>(entity).m_Bounds;var size=bounds.max-bounds.min;var footprint=Math.Max(size.x,size.z);
            if(!(footprint>0)||!math.all(math.isfinite(size))){log?.Count("Rock assets without usable size");continue;}
            // A pivot above the base (bounds.min.y < 0) is already partly sunk.
            found.Add((name,entity,footprint,Math.Max(0,.2f*size.y+Math.Min(0,bounds.min.y)),size));
        }
        var debris=found.Where(r=>r.Name.StartsWith("Rock",StringComparison.Ordinal)).OrderBy(r=>r.Footprint).ToArray();
        var mediumFrom=debris.Length<3?float.PositiveInfinity:debris[debris.Length*2/3].Footprint;
        var rocks=found.Select(r=>new StockPrefab(r.Name,r.Entity,
            r.Name.StartsWith("Boulder",StringComparison.Ordinal)?VegetationForm.RockLarge:r.Footprint>=mediumFrom?VegetationForm.RockMedium:VegetationForm.RockSmall,
            r.Sink,r.Footprint/2)).ToList();
        string Describe(VegetationForm form)
        {
            var group=rocks.Where(r=>r.Form==form).ToArray();
            return group.Length==0?"none":$"{group.Length}, {group.Min(r=>2*r.Radius):0.0}–{group.Max(r=>2*r.Radius):0.0} m across";
        }
        log?.Evidence($"Rock catalog: {rocks.Count} vanilla rocks; boulders {Describe(VegetationForm.RockLarge)}; larger rocks {Describe(VegetationForm.RockMedium)}; small rocks {Describe(VegetationForm.RockSmall)}. "+
            string.Join("; ",found.OrderBy(r=>r.Name,StringComparer.Ordinal).Select(r=>$"{r.Name} {r.Size.x:0.0}×{r.Size.z:0.0}×{r.Size.y:0.0} m")));
        return rocks;
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
    private readonly int[][] _palettes=new int[FormCount][];
    private readonly bool[] _valid;
    private readonly int[] _assetCounts;
    private readonly int[] _formCounts=new int[FormCount];
    private readonly EntityArchetype _archetype;
    private readonly Func<float3,bool>? _localRoadExclusion;
    private readonly List<Entity> _definitions=new(8192);
    private readonly IReadOnlyList<Entity> _view;
    public NativeVegetationPlacement(EntityManager manager,StockPrefab[] catalog,ImportLog log,Func<float3,bool>? localRoadExclusion=null)
    {
        _manager=manager;_log=log;_epoch=new ImportEpoch(ImportSession.Map,ImportSession.Generation,manager.World.SequenceNumber);
        _localRoadExclusion=localRoadExclusion;
        _catalog=(StockPrefab[])catalog.Clone();_valid=new bool[catalog.Length];_assetCounts=new int[catalog.Length];_view=_definitions.AsReadOnly();
        for(var form=0;form<FormCount;form++)
        {
            var indices=new List<int>();
            for(var i=0;i<_catalog.Length;i++)if((int)_catalog[i].Form==form&&!Placeholder(_catalog[i].Name))indices.Add(i);
            _palettes[form]=indices.ToArray();
        }
        // An empty rock size class borrows the nearest one, so a site is never lost
        // because the vanilla set happens to have no rock of that footprint.
        var rockForms=new[]{VegetationForm.RockSmall,VegetationForm.RockMedium,VegetationForm.RockLarge};
        var rockPalettes=rockForms.Select(f=>_palettes[(int)f]).ToArray();
        for(var r=0;r<rockForms.Length;r++)if(rockPalettes[r].Length==0)
            _palettes[(int)rockForms[r]]=Enumerable.Range(0,rockForms.Length).OrderBy(o=>Math.Abs(o-r)).Select(o=>rockPalettes[o]).FirstOrDefault(p=>p.Length>0)??Array.Empty<int>();
        _archetype=manager.CreateArchetype(ComponentType.ReadWrite<CreationDefinition>(),ComponentType.ReadWrite<ObjectDefinition>(),ComponentType.ReadWrite<Updated>());
        _rockCell=Math.Max(16,2*_catalog.Select(p=>p.Radius).DefaultIfEmpty(0).Max());
    }

    // Real footprints of rocks placed so far, across batches. Cluster anchors come
    // first in the plan, so a boulder claims its ground and debris that would sit
    // inside it is dropped. Touching and slight overlap (a quarter of the summed
    // radii) are allowed, as in a natural pile. A cell is at least the largest
    // two radii together, so the 3×3 scan sees every possible overlap.
    private readonly Dictionary<(int,int),List<float3>> _rockDiscs=new();
    private float _rockCell=16;
    private int _rockOverlaps;
    private bool RockFits(float3 position,float radius)
    {
        var cx=(int)Math.Floor(position.x/_rockCell);var cz=(int)Math.Floor(position.z/_rockCell);
        for(var x=cx-1;x<=cx+1;x++)for(var z=cz-1;z<=cz+1;z++)
            if(_rockDiscs.TryGetValue((x,z),out var discs))
                foreach(var d in discs)if(math.distance(d.xy,position.xz)<.75f*(d.z+radius)){_rockOverlaps++;return false;}
        if(!_rockDiscs.TryGetValue((cx,cz),out var cell))_rockDiscs[(cx,cz)]=cell=new();
        cell.Add(new float3(position.x,position.z,radius));return true;
    }
    // Undoes RockFits for a rock that was reserved but never created.
    private void ReleaseRock(float3 position,int index)
    {
        if(!IsRock(_catalog[index].Form))return;
        var key=((int)Math.Floor(position.x/_rockCell),(int)Math.Floor(position.z/_rockCell));
        if(_rockDiscs.TryGetValue(key,out var cell))cell.Remove(new float3(position.x,position.z,_catalog[index].Radius));
    }
    private void CountRockOverlaps(){if(_rockOverlaps>0){_log.Count("Rocks dropped inside a larger rock",_rockOverlaps);_rockOverlaps=0;}}

    // Plants keep their PlantData contract; rocks are static objects with geometry.
    private bool Usable(int index,bool direct)
    {
        var entity=_catalog[index].Entity;
        return _manager.Exists(entity)&&_manager.HasComponent<PrefabData>(entity)&&(!direct||_manager.HasComponent<ObjectData>(entity))&&
            (IsRock(_catalog[index].Form)?_manager.HasComponent<ObjectGeometryData>(entity):_manager.HasComponent<PlantData>(entity));
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
        _definitions.Clear();Array.Clear(_formCounts,0,FormCount);Array.Clear(_assetCounts,0,_assetCounts.Length);
        for(var i=0;i<_catalog.Length;i++)_valid[i]=Usable(i,direct:false);
        var failures=0;var nonfinite=0;var retired=0;var missing=new int[FormCount];
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
                if(form<0||form>=FormCount){_manager.DestroyEntity(entity);failures++;continue;}
                var palette=_palettes[form];
                if(palette.Length==0){_manager.DestroyEntity(entity);missing[form]++;continue;}
                var index=palette[seed%(uint)palette.Length];var selected=_catalog[index];
                if(!_valid[index]){_manager.DestroyEntity(entity);retired++;continue;}
                if(IsRock(selected.Form)&&!RockFits(point,selected.Radius)){_manager.DestroyEntity(entity);continue;}
                _manager.SetComponentData(entity,new CreationDefinition
                {
                    m_Prefab=selected.Entity,m_SubPrefab=Entity.Null,m_Original=Entity.Null,
                    m_Owner=Entity.Null,m_Attached=Entity.Null,m_Flags=0,m_RandomSeed=(int)seed
                });
                _manager.SetComponentData(entity,new ObjectDefinition
                {
                    m_Position=point-new float3(0,selected.Sink,0),m_LocalPosition=float3.zero,m_Scale=new float3(1),
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
        for(var form=0;form<FormCount;form++)
        {
            if(_formCounts[form]>0)_log.Count("Vegetation definitions "+(VegetationForm)form,_formCounts[form]);
            if(missing[form]>0)_log.Count("Vegetation unavailable stock palette "+(VegetationForm)form,missing[form]);
        }
        for(var i=0;i<_catalog.Length;i++)if(_assetCounts[i]>0)_log.Count("Vegetation definitions asset "+_catalog[i].Name,_assetCounts[i]);
        if(nonfinite>0)_log.Count("Vegetation invalid coordinates omitted",nonfinite);
        if(retired>0)_log.Count("Vegetation retired prefabs omitted",retired);
        CountRockOverlaps();
        if(failures>0){_log.Count("Trees skipped",failures);_log.Count("Vegetation definition failures",failures);}
        NetworkCommitSystem.DescribeFoliageDefinitions(count,_definitions.Count);
        if(_log.DiagnosticsEnabled)_log.Diagnostic($"VEGETATION ALLOCATION requested={count}; definitions={_definitions.Count}; bulkArchetypeAllocation=true; nativeInitializationPreserved=true; catalogCached=true");
        if(_definitions.Count==0)NetworkCommitSystem.ReleaseEmptyDefinitions(_manager);
        return _view; // Remains unchanged until settlement and the next CreateBatch.
    }

    // Finished vegetation from each prefab's own archetype (ObjectData.m_Archetype,
    // which already carries Created/Updated), set as Game.Debug.TreeSpawnSystem and
    // the tool's final object creation set it: PrefabRef, Transform, Tree state and
    // PseudoRandomSeed. No definitions, Temp previews or apply pass (report 67).
    // Candidate checks and stock selection are identical to CreateBatch.
    // Progress of the latest CreateDirect, so a failure mid-batch reports exactly
    // how much was created, plus a small per-prefab sample for health checks.
    public int DirectCreated { get; private set; }
    // Unfinished entities removed after a failure in the latest CreateDirect.
    public int DirectRemoved { get; private set; }
    public List<Entity> DirectSample { get; } = new(32);
    public int CreateDirect(IReadOnlyList<VegetationCandidate> points,int start,int count)
    {
        DirectCreated=0;DirectRemoved=0;DirectSample.Clear();
        if(start<0||count<0||start>points.Count-count)throw new ArgumentOutOfRangeException(nameof(start));
        if(!_manager.World.IsCreated||!ImportSession.Matches(_epoch,_manager.World.SequenceNumber))
            throw new InvalidOperationException("Vegetation catalog belongs to a retired world/import.");
        Array.Clear(_formCounts,0,FormCount);Array.Clear(_assetCounts,0,_assetCounts.Length);
        for(var i=0;i<_catalog.Length;i++)_valid[i]=Usable(i,direct:true);
        var groups=new List<int>[_catalog.Length];var nonfinite=0;var retired=0;var failures=0;var excluded=0;var missing=new int[FormCount];
        for(var offset=0;offset<count;offset++)
        {
            var candidate=points[start+offset];
            if(!math.all(math.isfinite(candidate.Position))){nonfinite++;continue;}
            if(_localRoadExclusion?.Invoke(candidate.Position)==true){excluded++;continue;}
            var form=(int)candidate.Form;if(form<0||form>=FormCount){failures++;continue;}
            var palette=_palettes[form];if(palette.Length==0){missing[form]++;continue;}
            var index=palette[candidate.Seed%(uint)palette.Length];if(!_valid[index]){retired++;continue;}
            if(IsRock(_catalog[index].Form)&&!RockFits(candidate.Position,_catalog[index].Radius))continue;
            (groups[index]??=new List<int>()).Add(start+offset);_formCounts[form]++;_assetCounts[index]++;
        }
        var created=0;var adult=Game.Objects.ObjectUtils.InitializeTreeState(1f);
        for(var index=0;index<groups.Length;index++)
        {
            var group=groups[index];if(group==null)continue;var prefab=_catalog[index].Entity;var sink=new float3(0,_catalog[index].Sink,0);
            var archetype=_manager.GetComponentData<ObjectData>(prefab).m_Archetype;var tree=_manager.HasComponent<TreeData>(prefab);
            using var entities=new NativeArray<Entity>(group.Count,Allocator.Temp);
            _manager.CreateEntity(archetype,entities);
            var k=0;
            try
            {
                var seeded=_manager.HasComponent<PseudoRandomSeed>(entities[0]);
                for(;k<group.Count;k++)
                {
                    var candidate=points[group[k]];var entity=entities[k];var seed=candidate.Seed;
                    _manager.SetComponentData(entity,new PrefabRef(prefab));
                    _manager.SetComponentData(entity,new Game.Objects.Transform(candidate.Position-sink,quaternion.RotateY(seed/(float)uint.MaxValue*math.PI*2)));
                    if(tree)_manager.SetComponentData(entity,adult);
                    if(seeded)_manager.SetComponentData(entity,new PseudoRandomSeed((ushort)(seed&0xFFFF)));
                }
            }
            catch(Exception error) when(error is not OutOfMemoryException)
            {
                // Entities written in full stay; this group's unfinished suffix (the
                // one being written included) is removed by its own handles, and
                // rock spacing reserved for candidates never created is released,
                // so standard planting of the remainder sees no phantom rocks.
                created+=k;DirectCreated=created;
                var removed=0;
                for(var j=k;j<group.Count;j++)
                    try{if(_manager.Exists(entities[j])){_manager.DestroyEntity(entities[j]);removed++;}}
                    catch(Exception cleanup) when(cleanup is not OutOfMemoryException){_log.Warn($"Could not remove unfinished tree {group[j]}: {cleanup.Message}");}
                DirectRemoved=removed;
                for(var j=k;j<group.Count;j++)ReleaseRock(points[group[j]].Position,index);
                for(var later=index+1;later<groups.Length;later++)if(groups[later]!=null)foreach(var i in groups[later])ReleaseRock(points[i].Position,later);
                throw new InvalidOperationException($"Direct vegetation stopped at {created:N0} created; {removed:N0} unfinished entities removed.",error);
            }
            created+=group.Count;DirectCreated=created;
            for(var j=0;j<Math.Min(2,group.Count)&&DirectSample.Count<32;j++)DirectSample.Add(entities[j]);
        }
        for(var form=0;form<FormCount;form++)
        {
            if(_formCounts[form]>0)_log.Count("Vegetation definitions "+(VegetationForm)form,_formCounts[form]);
            if(missing[form]>0)_log.Count("Vegetation unavailable stock palette "+(VegetationForm)form,missing[form]);
        }
        for(var i=0;i<_catalog.Length;i++)if(_assetCounts[i]>0)_log.Count("Vegetation definitions asset "+_catalog[i].Name,_assetCounts[i]);
        if(excluded>0)_log.Count("Vegetation excluded by revised road footprint",excluded);
        if(nonfinite>0)_log.Count("Vegetation invalid coordinates omitted",nonfinite);
        if(retired>0)_log.Count("Vegetation retired prefabs omitted",retired);
        CountRockOverlaps();
        if(failures>0){_log.Count("Trees skipped",failures);_log.Count("Vegetation definition failures",failures);}
        return created;
    }
}
