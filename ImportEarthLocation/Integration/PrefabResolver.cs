using System;
using System.Collections.Generic;
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
    // Resolver lifetime is one validation/application phase, never shared across maps.
    private readonly Dictionary<(Entity,bool),float> _widthCache = new();
    private readonly Dictionary<Entity,float[]> _laneOffsetCache = new();
    private RoadPrefab[]? _roadCatalog;
    private TrackPrefab[]? _trackCatalog;
    private readonly Dictionary<int,Entity> _railPrefabs = new();
    public Dictionary<string,float[]> LaneOffsets { get; } = new();

    public PrefabResolver(PrefabSystem prefabSystem, ImportLog log)
    {
        _prefabSystem = prefabSystem ?? throw new ArgumentNullException(nameof(prefabSystem));
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
        var ok = ResolveRail(2) != Entity.Null;
        error = ok ? "" : "No usable vanilla two-track prefab could be resolved; network application is blocked before map replacement.";
        return ok;
    }

    public Entity Resolve(PreparedSegment segment)
    {
        var key = $"{segment.Kind}:{SizedPaletteKey(segment)}";
        if (_cache.TryGetValue(key, out var cached)) return cached;
        Entity entity;
        if (segment.Kind == NetworkKind.Rail)
        {
            entity = ResolveRail(segment.PhysicalTracks > 0 ? segment.PhysicalTracks : segment.Lanes);
        }
        else
        {
            entity = ResolveRoad(segment);
        }
        _cache[key] = entity;
        if (entity == Entity.Null) _log.Warn($"No prefab matched {key}; complete network preflight must succeed before replacing the map.");
        return entity;
    }

    private Entity ResolveRail(int tracks)
    {
        tracks=Numeric.Clamp(tracks,1,2);
        if (_railPrefabs.TryGetValue(tracks,out var cachedRail)) return cachedRail;
        try
        {
        _trackCatalog ??= NativePrefabCatalog.Read<TrackPrefab>(_prefabSystem.EntityManager,_prefabSystem).ToArray();
        // Parallel OSM tracks were consolidated to one double-track corridor axis.
        var candidates=_trackCatalog.Where(p=>p.m_TrackType==Game.Net.TrackTypes.Train && RailPrefabPolicy.IsBaseCorridor(p.name))
            .Select(p=>new{Prefab=p,Entity=_prefabSystem.GetEntity(p)})
            .Where(p=>ReadWidth(p.Entity)>0&&!float.IsInfinity(ReadWidth(p.Entity))&&!float.IsNaN(ReadWidth(p.Entity)))
            .OrderBy(p=>ReadWidth(p.Entity)).ThenBy(p=>p.Prefab.name,StringComparer.Ordinal).ToArray();
        // Width and bidirectional names cannot prove the number of physical tracks.
        foreach(var candidate in candidates.OrderBy(p=>p.Prefab.name=="Double Train Track"?0:1))
        {
            var count=TrackCount(candidate.Entity);
            var readback = "native-default-lanes";
            if(count <= 0) { count = DefinitionTrackCount(candidate.Prefab); readback = "prefab-section-piece-lanes"; }
            _log.Diagnostic($"RAIL PREFAB candidate={candidate.Prefab.name}; widthMeters={ReadWidth(candidate.Entity):0.##}; physicalTracks={count}; laneReadback={readback}; nativeLaneReadback={count >= 0}");
            // Section-local offsets are not composition coordinates: native layout
            // packs sections side by side. Deduplicating all section-local x=0
            // lanes incorrectly turned the vanilla double corridor into one track.
            // Only the exact installed vanilla double-track asset may use the
            // layout fallback; "Twoway" is a single bidirectional physical track.
            var layoutFallback = candidate.Prefab.name == "Double Train Track" &&
                readback == "prefab-section-piece-lanes" && count == 1 && ReadWidth(candidate.Entity) >= 10;
            var actualCount=layoutFallback?2:count;
            if (!RailPrefabPolicy.CanUse(candidate.Prefab.name, actualCount) || actualCount!=tracks) continue;
            _log.Diagnostic($"RAIL PREFAB selected={candidate.Prefab.name}; physicalTracks={(layoutFallback?2:count)}; namedFallback={layoutFallback}; physicalCountVerified={!layoutFallback}; compositionLayoutFallback={layoutFallback}");
            if(layoutFallback)_log.Warn("Double Train Track selected by exact vanilla asset and native width: section-local coordinates cannot prove composed track count; live two-track rendering must be checked.");
            _railPrefabs[tracks] = candidate.Entity;
            return candidate.Entity;
        }
        }
        catch (Exception exception) when (exception is not OutOfMemoryException && exception is not OperationCanceledException)
        { _log.Warn($"RAIL PREFAB lookup failed: {exception.GetType().Name}: {exception.Message}. Network application cannot continue without the requested railway."); }
        _railPrefabs[tracks] = Entity.Null;
        return Entity.Null;
    }

    // DefaultNetLane need not exist on the top-level TrackPrefab entity. Native
    // definitions instead own sections, pieces and NetPieceLanes components.
    // Count distinct lateral positions including section/piece offsets; repeated
    // ground/bridge variants at the same position do not create additional tracks.
    private static int DefinitionTrackCount(TrackPrefab prefab)
    {
        var visited = new HashSet<NetSectionPrefab>();
        var allOffsets = new List<float>();
        void Visit(NetSectionPrefab section, float offset, float direction)
        {
            if(section == null || !visited.Add(section)) return;
            if(section.m_Pieces != null) foreach(var info in section.m_Pieces)
            {
                if(info.m_Piece == null || !info.m_Piece.TryGet<NetPieceLanes>(out var lanes) || lanes.m_Lanes == null) continue;
                var offsets = new List<float>();
                foreach(var lane in lanes.m_Lanes)
                {
                    if(lane.m_Lane == null || !lane.m_Lane.TryGet<TrackLane>(out var track) || (track.m_TrackType & Game.Net.TrackTypes.Train) == 0) continue;
                    var x=offset+direction*(info.m_Offset.x+lane.m_Position.x);
                    if(!offsets.Any(v=>Math.Abs(v-x)<.2f)) offsets.Add(x);
                }
                foreach(var x in offsets) if(!allOffsets.Any(v=>Math.Abs(v-x)<.2f)) allOffsets.Add(x);
            }
            if(section.m_SubSections != null) foreach(var child in section.m_SubSections) Visit(child.m_Section,offset,direction);
            visited.Remove(section);
        }
        if(prefab.m_Sections != null) foreach(var section in prefab.m_Sections) Visit(section.m_Section,section.m_Offset.x,section.m_Invert?-1f:1f);
        return allOffsets.Count==0?-1:allOffsets.Count;
    }

    private int TrackCount(Entity entity)
    {
        var manager=_prefabSystem.EntityManager;if(!manager.HasBuffer<DefaultNetLane>(entity))return -1;
        var offsets=new List<float>();
        foreach(var lane in manager.GetBuffer<DefaultNetLane>(entity,true))
        {
            if(!manager.HasComponent<TrackLaneData>(lane.m_Lane)||(manager.GetComponentData<TrackLaneData>(lane.m_Lane).m_TrackTypes&Game.Net.TrackTypes.Train)==0)continue;
            if(!offsets.Any(x=>Math.Abs(x-lane.m_Position.x)<.2f))offsets.Add(lane.m_Position.x);
        }
        return offsets.Count;
    }

    private static bool IsHighway(PreparedSegment s) => s.Classification.StartsWith("motorway",StringComparison.OrdinalIgnoreCase)||s.Classification.StartsWith("trunk",StringComparison.OrdinalIgnoreCase);
    public static string PaletteKey(PreparedSegment s) => $"{(IsHighway(s)?"highway":"road")}:{s.OneWay}:{s.Bridge}:{s.ConsolidatedAvenue}";
    // Ordinary one-way roads can contain six lanes (including turn lanes). Only
    // the one-way highway palette is capped at five. Snapshot and lookup must agree.
    public static string SizedPaletteKey(PreparedSegment s) => s.Kind==NetworkKind.Rail ? $"rail:{s.Bridge}:{Numeric.Clamp(s.PhysicalTracks>0?s.PhysicalTracks:s.Lanes,1,2)}" : PaletteKey(s)+":"+Numeric.Clamp(s.Lanes,1,IsHighway(s)&&s.OneWay?5:6);
    public static PreparedSegment ApplyMetadata(PreparedSegment s,IReadOnlyDictionary<string,float> widths,IReadOnlyDictionary<string,float[]> offsets)
    {
        var key=SizedPaletteKey(s);
        return s with{NativeWidth=widths.TryGetValue(key,out var width)&&!float.IsNaN(width)&&!float.IsInfinity(width)&&width>0?width:0,
            NativeLaneOffsets=s.Kind==NetworkKind.Rail?Array.Empty<float>():offsets.TryGetValue(key,out var lanes)?lanes:Array.Empty<float>()};
    }

    public PreparedSegment ResolveMetadata(PreparedSegment s)
    {
        var entity=Resolve(s);
        if(entity==Entity.Null)throw new InvalidOperationException($"No verified vanilla prefab for {s.Kind} way {s.WayId}; all requested sections must remain present.");
        var width=ReadWidth(entity,s.Bridge);
        if(float.IsNaN(width)||float.IsInfinity(width)||width<=0)
            throw new InvalidOperationException($"Invalid native width for {s.Kind} way {s.WayId}: {width}.");
        var offsets=s.NativeLaneOffsets;
        if(s.Kind==NetworkKind.Road)
        {
            try{var readback=ReadCarLaneOffsets(entity);if(readback.Length>0)offsets=readback;}
            catch(Exception error) when(error is not OutOfMemoryException&&error is not OperationCanceledException)
            { _log.Warn($"Native lane metadata for way {s.WayId} could not be refreshed: {error.Message}. Previously captured lane positions retained."); }
        }
        return s with{NativeWidth=width,NativeLaneOffsets=offsets};
    }
    public Entity ResolveNarrowest(PreparedSegment segment)
    {
        _roadCatalog ??= NativePrefabCatalog.Read<RoadPrefab>(_prefabSystem.EntityManager,_prefabSystem).ToArray();
        var highway=IsHighway(segment);
        foreach(var p in _roadCatalog.Where(p=>p.m_HighwayRules==highway&&NetworkSelection.IsBasicVanillaRoad(p.name))
            .OrderBy(p=>ReadWidth(_prefabSystem.GetEntity(p),segment.Bridge)).ThenBy(p=>p.name,StringComparer.Ordinal))
        {var entity=_prefabSystem.GetEntity(p);if(Suitable(entity,segment))return entity;}
        return Entity.Null;
    }
    public Dictionary<string,float> CaptureMinimumWidths()
    {
        var result=new Dictionary<string,float>();
        foreach(var bridge in new[]{false,true})foreach(var tracks in new[]{1,2})
        {
            var rail=new PreparedSegment(0,1,2,default,default,NetworkKind.Rail,"rail",tracks,false,bridge,false,0){PhysicalTracks=tracks};
            var entity=ResolveRail(tracks);result[SizedPaletteKey(rail)]=entity==Entity.Null?float.PositiveInfinity:ReadWidth(entity,bridge);
        }
        foreach(var highway in new[]{false,true})foreach(var oneWay in new[]{false,true})foreach(var bridge in new[]{false,true})foreach(var avenue in new[]{false,true})
        {
            var s=new PreparedSegment(0,1,2,default,default,NetworkKind.Road,highway?"motorway":"primary",oneWay?1:2,oneWay,bridge,false,0){ConsolidatedAvenue=avenue};
            var entity=ResolveNarrowest(s);result[PaletteKey(s)]=entity==Entity.Null?float.PositiveInfinity:ReadWidth(entity,bridge);
            for(var lanes=1;lanes<=(highway&&oneWay?5:6);lanes++)
            {
                var requested=s with{Lanes=lanes};var sized=ResolveRoad(requested);
                result[PaletteKey(s)+":"+lanes]=sized==Entity.Null?float.PositiveInfinity:ReadWidth(sized,bridge);
                try{LaneOffsets[PaletteKey(s)+":"+lanes]=ReadCarLaneOffsets(sized);}
                catch(Exception exception) when(exception is not OutOfMemoryException&&exception is not OperationCanceledException)
                {LaneOffsets[PaletteKey(s)+":"+lanes]=Array.Empty<float>();_log.Warn($"Lane-position metadata unavailable for {PaletteKey(s)}:{lanes}: {exception.Message}. Direction-only ramp smoothing retained; preparation continues.");}
            }
        }
        return result;
    }
    private float[] ReadCarLaneOffsets(Entity entity)
    {
        if(_laneOffsetCache.TryGetValue(entity,out var offsets))return offsets;
        try{return _laneOffsetCache[entity]=CaptureCarLaneOffsets(entity);}
        catch(Exception error) when(error is not OutOfMemoryException&&error is not OperationCanceledException)
        {
            _laneOffsetCache[entity]=Array.Empty<float>();
            _log.Warn($"Native lane-position readback unavailable: {error.Message}. Captured positions or bounded lane-spacing estimates retained; no network sections omitted.");
            return _laneOffsetCache[entity];
        }
    }
    private float[] CaptureCarLaneOffsets(Entity entity)
    {
        if(entity==Entity.Null)return Array.Empty<float>();
        var manager=_prefabSystem.EntityManager;var offsets=new List<float>();
        if(manager.HasBuffer<DefaultNetLane>(entity))foreach(var lane in manager.GetBuffer<DefaultNetLane>(entity,true))
        {if(manager.HasComponent<CarLaneData>(lane.m_Lane)&&!float.IsNaN(lane.m_Position.x)&&!float.IsInfinity(lane.m_Position.x)&&!offsets.Any(x=>Math.Abs(x-lane.m_Position.x)<.2f))offsets.Add(lane.m_Position.x);}
        var prefab=_prefabSystem.GetPrefab<RoadPrefab>(entity);var visited=new HashSet<NetSectionPrefab>();
        var expected=NamedLaneCount(prefab.name);
        if(offsets.Count>0&&(expected==0||offsets.Count==expected))return offsets.OrderBy(x=>x).ToArray();
        offsets.Clear();
        void Visit(NetSectionPrefab section,float offset,float direction)
        {
            if(section==null||!visited.Add(section))return;
            if(section.m_Pieces!=null)foreach(var info in section.m_Pieces)
            {
                if(info.m_Piece==null||!info.m_Piece.TryGet<NetPieceLanes>(out var lanes)||lanes.m_Lanes==null)continue;
                foreach(var lane in lanes.m_Lanes)
                {
                    var x=offset+direction*(info.m_Offset.x+lane.m_Position.x);
                    if(lane.m_Lane!=null&&lane.m_Lane.TryGet<CarLane>(out _)&&!float.IsNaN(x)&&!float.IsInfinity(x)&&!offsets.Any(v=>Math.Abs(v-x)<.2f))offsets.Add(x);
                }
            }
            if(section.m_SubSections!=null)foreach(var child in section.m_SubSections)Visit(child.m_Section,offset,direction);
            visited.Remove(section);
        }
        if(prefab.m_Sections!=null)foreach(var section in prefab.m_Sections)Visit(section.m_Section,section.m_Offset.x,section.m_Invert?-1:1);
        if(expected>0&&offsets.Count!=expected)
        {
            // Local section coordinates cannot describe the packed composition.
            // Use the selected vanilla asset's count, not the OSM requested count.
            _log.Diagnostic($"LANE PALETTE selected={prefab.name}; requestedCompositionLanes={expected}; localDistinctOffsets={offsets.Count}; composedSpacingEstimated=true; liveLanePositionsRequireVerification=true");
            return Enumerable.Range(0,expected).Select(i=>(i-(expected-1)*.5f)*4).ToArray();
        }
        return offsets.OrderBy(x=>x).ToArray();
    }

    private static int NamedLaneCount(string name)
    {
        if(name=="Small Road")return 2;if(name=="Medium Road")return 4;if(name=="Large Road")return 6;
        foreach(var count in Enumerable.Range(1,6))if(name.EndsWith($"- {count} lane{(count==1?"":"s")}",StringComparison.Ordinal))return count;
        return 0;
    }

    private Entity ResolveRoad(PreparedSegment segment)
    {
        // Spacing estimates drive geometry repair, not asset selection. Applying
        // them here made catalog fallbacks differ from the preparation snapshot
        // and rejected usable ordinary one-way roads at e.g. 15.99 versus 16 m.
        var estimatedWidth=segment.MaximumNativeWidth;
        segment=segment with{MaximumNativeWidth=float.PositiveInfinity,Lanes=Numeric.Clamp(segment.Lanes,1,IsHighway(segment)&&segment.OneWay?5:6)};
        var highway = segment.Classification.StartsWith("motorway", StringComparison.OrdinalIgnoreCase) ||
                      segment.Classification.StartsWith("trunk", StringComparison.OrdinalIgnoreCase);
        var candidates = new List<string>();
        if (highway && segment.OneWay)
        {
            var nativeLanes=Numeric.Clamp(segment.Lanes,1,5);
            candidates.Add($"Highway Oneway - {nativeLanes} lane{(nativeLanes == 1 ? "" : "s")}");
        }
        else if (highway)
        {
            candidates.Add($"Highway Twoway - {Math.Max(2,segment.Lanes)} lanes");
            candidates.Add("Highway");
        }
        else if (segment.OneWay)
        {
            var size=segment.Lanes<=2?"Small":segment.Lanes<=4?"Medium":"Large";
            candidates.Add($"{size} Road Oneway - {segment.Lanes} lane{(segment.Lanes == 1 ? "" : "s")}");
        }
        else
        {
            candidates.Add(NetworkSelection.BasicRoadName(segment.Lanes));
        }
        if(segment.ConsolidatedAvenue) { candidates.Add("Medium Road"); candidates.Add("Large Road"); }
        else if(!segment.OneWay&&!highway)candidates.Add("Small Road");
        // The requested basic size wins over a smaller palette entry. A narrow
        // OSM corridor is not evidence for reducing the requested lane count.
        foreach(var name in candidates.Take(1))
        {
            var candidate=ResolveFirst<RoadPrefab>(new[]{name});
            if(candidate!=Entity.Null && Suitable(candidate,segment))
            {
                var nativeCount=NamedLaneCount(name);
                if(nativeCount>0&&nativeCount!=segment.Lanes)
                    _log.Diagnostic($"ROAD PREFAB vanilla-lane-approximation requestedLanes={segment.Lanes}; selected={name}; nativeLanes={nativeCount}; exactSourceSizeAvailable=false");
                if(ReadWidth(candidate,segment.Bridge)>estimatedWidth+.01f)
                {
                    _log.Count("Road native width constraints relaxed");
                    _log.Diagnostic($"ROAD PREFAB size-preserved classification={segment.Classification}; requestedLanes={segment.Lanes}; selected={name}; estimatedWidth={estimatedWidth}; nativeWidth={ReadWidth(candidate,segment.Bridge)}; oneWay={segment.OneWay}");
                }
                return candidate;
            }
        }
        // Stay in the basic vanilla palette. Catalog-wide width matching used to
        // select decorated/special-purpose/custom roads and vary types unpredictably.
        _roadCatalog ??= NativePrefabCatalog.Read<RoadPrefab>(_prefabSystem.EntityManager,_prefabSystem).ToArray();
        if(highway&&segment.OneWay)
        {
            foreach(var lanes in Enumerable.Range(1,5).OrderBy(n=>Math.Abs(n-Numeric.Clamp(segment.Lanes,1,5))).ThenByDescending(n=>n))
            {
                var name=$"Highway Oneway - {lanes} lane{(lanes==1?"":"s")}";
                var entity=ResolveFirst<RoadPrefab>(new[]{name});
                if(entity==Entity.Null||!Suitable(entity,segment with{MaximumNativeWidth=float.PositiveInfinity}))continue;
                _log.Diagnostic($"ROAD PREFAB supported-size-fallback requestedLanes={segment.Lanes}; selected={name}; sourceSizeUnavailable=true; widthEstimateIgnored=true");
                return entity;
            }
        }
        foreach(var prefab in _roadCatalog
            .Where(p=>p.m_HighwayRules==highway&&NetworkSelection.IsBasicVanillaRoad(p.name)&&(!segment.ConsolidatedAvenue||p.name==NetworkSelection.BasicRoadName(segment.Lanes)))
            .OrderBy(p=>NamedLaneCount(p.name)==0?100:Math.Abs(NamedLaneCount(p.name)-segment.Lanes)).ThenByDescending(p=>NamedLaneCount(p.name)).ThenBy(p=>Math.Abs(ReadWidth(_prefabSystem.GetEntity(p))-segment.Lanes*3.5f)).ThenBy(p=>p.name,StringComparer.Ordinal))
        {var entity=_prefabSystem.GetEntity(prefab);if(Suitable(entity,segment)){_log.Diagnostic($"ROAD PREFAB lane-count-fallback requestedLanes={segment.Lanes}; selected={prefab.name}; nativeNamedLanes={NamedLaneCount(prefab.name)}; directionPreserved=true");return entity;}}
        return Entity.Null;
    }

    public float ReadWidth(Entity entity, bool elevated = false)
    {
        if(_widthCache.TryGetValue((entity,elevated),out var cached))return cached;
        if(!_prefabSystem.EntityManager.HasComponent<NetGeometryData>(entity))return float.PositiveInfinity;
        var geometry=_prefabSystem.EntityManager.GetComponentData<NetGeometryData>(entity);
        var width=elevated?Math.Max(geometry.m_DefaultWidth,geometry.m_ElevatedWidth):geometry.m_DefaultWidth;
        _widthCache[(entity,elevated)]=width;return width;
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
