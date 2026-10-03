using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Game.Prefabs;
using ImportEarthLocation.Core;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

public sealed class PrefabResolver
{
    private readonly PrefabSystem _prefabSystem;
    private readonly ImportLog _log;
    private readonly Dictionary<string, Entity> _cache = new(StringComparer.OrdinalIgnoreCase);
    private RoadPrefab[]? _roadCatalog;
    private TrackPrefab[]? _trackCatalog;

    public PrefabResolver(PrefabSystem prefabSystem, ImportLog log)
    {
        _prefabSystem = prefabSystem;
        _log = log;
    }

    public bool CanResolveRoad(out string error)
    {
        var ok = ResolveFirst<RoadPrefab>(new[] { "Small Road", "Medium Road", "Highway Twoway - 2 lanes" }) != Entity.Null;
        error = ok ? "" : "No supported vanilla road prefab was found.";
        return ok;
    }

    public bool CanResolveRail(out string error)
    {
        var ok = ResolveRail(1) != Entity.Null;
        error = ok ? "" : "No supported vanilla train-track prefab was found.";
        return ok;
    }

    public Entity Resolve(PreparedSegment segment)
    {
        var key = $"{segment.Kind}:{segment.Classification}:{segment.Lanes}:{segment.OneWay}:{segment.Bridge}:{segment.Tunnel}:{segment.MaximumNativeWidth.ToString("R",CultureInfo.InvariantCulture)}";
        if (_cache.TryGetValue(key, out var cached)) return cached;
        Entity entity;
        if (segment.Kind == NetworkKind.Rail)
        {
            entity = ResolveRail(segment.Lanes);
        }
        else
        {
            entity = ResolveRoad(segment);
        }
        _cache[key] = entity;
        if (entity == Entity.Null) _log.Warn($"No prefab matched {key}; its segments will be skipped.");
        return entity;
    }

    private Entity ResolveRail(int tracks)
    {
        _trackCatalog ??= NativePrefabCatalog.Read<TrackPrefab>(_prefabSystem.EntityManager,_prefabSystem).ToArray();
        // An OSM track centerline must not silently fall back to a double-track
        // corridor. Prefer the narrowest verified native train geometry for it.
        var candidates=_trackCatalog.Where(p=>p.m_TrackType==Game.Net.TrackTypes.Train)
            .Select(p=>new{Prefab=p,Entity=_prefabSystem.GetEntity(p)})
            .Where(p=>ReadWidth(p.Entity)>0&&!float.IsInfinity(ReadWidth(p.Entity))&&!float.IsNaN(ReadWidth(p.Entity)))
            .OrderBy(p=>ReadWidth(p.Entity)).ThenBy(p=>p.Prefab.name,StringComparer.Ordinal).ToArray();
        if(tracks>1)
        {
            var named=ResolveFirst<TrackPrefab>(new[]{"Double Train Track"});
            if(named!=Entity.Null&&candidates.Any(p=>p.Entity==named))return named;
        }
        return candidates.Length==0?Entity.Null:candidates[0].Entity;
    }

    private Entity ResolveRoad(PreparedSegment segment)
    {
        var highway = segment.Classification.StartsWith("motorway", StringComparison.OrdinalIgnoreCase) ||
                      segment.Classification.StartsWith("trunk", StringComparison.OrdinalIgnoreCase);
        var candidates = new List<string>();
        if (highway && segment.OneWay)
        {
            candidates.Add($"Highway Oneway - {Numeric.Clamp(segment.Lanes, 1, 3)} lane{(segment.Lanes == 1 ? "" : "s")}");
            candidates.Add("Highway Oneway - 2 lanes");
        }
        else if (highway)
        {
            candidates.Add("Highway Twoway - 2 lanes");
            candidates.Add("Highway");
        }
        else if (segment.OneWay)
        {
            candidates.Add($"Small Road Oneway - {Numeric.Clamp(segment.Lanes, 1, 2)} lane{(segment.Lanes == 1 ? "" : "s")}");
        }
        else
        {
            candidates.Add(segment.Lanes >= 4 ? "Large Road" : segment.Lanes >= 3 ? "Medium Road" : "Small Road");
        }
        if(!segment.OneWay)candidates.Add("Small Road");
        foreach(var name in candidates)
        {
            var candidate=ResolveFirst<RoadPrefab>(new[]{name});
            if(candidate!=Entity.Null && Suitable(candidate,segment))return candidate;
        }
        // Name-independent fallback uses verified native direction and width, never a
        // two-way boulevard for an independently represented one-way carriageway.
        _roadCatalog ??= NativePrefabCatalog.Read<RoadPrefab>(_prefabSystem.EntityManager,_prefabSystem).ToArray();
        foreach(var prefab in _roadCatalog
            .Where(p=>p.m_HighwayRules==highway).OrderBy(p=>Math.Abs(ReadWidth(_prefabSystem.GetEntity(p))-segment.Lanes*3.5f)).ThenBy(p=>p.name,StringComparer.Ordinal))
        {var entity=_prefabSystem.GetEntity(prefab);if(Suitable(entity,segment))return entity;}
        return Entity.Null;
    }

    private float ReadWidth(Entity entity, bool elevated = false)
    {
        if(!_prefabSystem.EntityManager.HasComponent<NetGeometryData>(entity))return float.PositiveInfinity;
        var geometry=_prefabSystem.EntityManager.GetComponentData<NetGeometryData>(entity);
        return elevated?Math.Max(geometry.m_DefaultWidth,geometry.m_ElevatedWidth):geometry.m_DefaultWidth;
    }
    private bool Suitable(Entity entity,PreparedSegment segment)
    {
        var manager=_prefabSystem.EntityManager;
        if(!manager.HasComponent<RoadData>(entity))return false;
        var flags=manager.GetComponentData<RoadData>(entity).m_Flags;
        var forward=(flags&RoadFlags.DefaultIsForward)!=0;var backward=(flags&RoadFlags.DefaultIsBackward)!=0;
        if(!NetworkSelection.NativeDirectionMatches(segment.OneWay,forward,backward))return false;
        var width=ReadWidth(entity,segment.Bridge);
        return !float.IsNaN(width)&&!float.IsInfinity(width)&&width>0&&width<=segment.MaximumNativeWidth;
    }

    private Entity ResolveFirst<T>(IEnumerable<string> names) where T : PrefabBase
    {
        foreach (var name in names)
        {
            // 1.6.x only offers TryGetPrefab(PrefabID, out PrefabBase); the generic
            // overloads take PrefabData/Entity/PrefabRef, not PrefabID.
            if (_prefabSystem.TryGetPrefab(new PrefabID(typeof(T).Name, name), out PrefabBase basePrefab) &&
                basePrefab is T prefab && prefab != null)
                return _prefabSystem.GetEntity(prefab);
        }
        return Entity.Null;
    }
}
