using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Game.Common;
using Game.Net;
using Game.Objects;
using Game.Prefabs;
using Game.Routes;
using Game.UI;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace ImportEarthLocation.Integration;

public static class NativeAirBridge
{
    public static string Apply(PreparedImport prepared, EntityManager manager, PrefabSystem prefabs, AirwaySystem airways, NameSystem names, ImportLog log)
    {
        // Read the actual native grid; do not assume MapExt's playable perimeter also changes airways.
        log.Diagnostic($"AIR BEGIN requested={prepared.AirDestinations.Count}; stage=airway-grid");
        var grid = airways.GetAirwayData().airplaneMap;
        var field = typeof(AirwayHelpers.AirwayMap).GetField("m_GridSize", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        if (!(field?.GetValue(grid) is int2 gridSize) || gridSize.x < 2 || gridSize.y < 2 || gridSize.x != gridSize.y)
            return "Air connections skipped: native airway perimeter could not be verified.";
        // Grid size counts cells; the outermost node index is gridSize, not gridSize - 1.
        var a = grid.GetNodePosition(new int2(0)); var b = grid.GetNodePosition(gridSize);
        if (!math.all(math.isfinite(a)) || !math.all(math.isfinite(b)) || math.abs(a.x + b.x) > 2 || math.abs(a.z + b.z) > 2 || math.abs(a.x - a.z) > 2)
            return "Air connections skipped: native airway bounds are not a verified centered square.";
        var half = Math.Abs(a.x);
        if (half <= 0 || half > prepared.Request.MapSize.WidthMeters / 2 + 10)
            return "Air connections skipped: native airway grid does not fit the selected map.";
        Entity prefabEntity = Entity.Null; ObjectData objectData = default; var selectedPrefabName="";
        log.Diagnostic($"AIR stage=prefab-catalog; nativeGrid={gridSize}; perimeterHalfMeters={half:0.###}");
        foreach (var prefab in NativePrefabCatalog.Read<ObjectPrefab>(manager, prefabs))
        {
            if (!prefab.TryGet<Game.Prefabs.OutsideConnection>(out var connection) || connection.m_TransferType != OutsideConnectionTransferType.Air ||
                !prefab.TryGet<Game.Prefabs.TransportStop>(out var stop) || stop.m_TransportType != TransportType.Airplane || !prefab.TryGet<Game.Prefabs.TakeoffLocation>(out _)) continue;
            var entity = prefabs.GetEntity(prefab);
            if (!manager.HasComponent<ObjectData>(entity)) continue;
            var data = manager.GetComponentData<ObjectData>(entity);
            using var components = data.m_Archetype.GetComponentTypes(Allocator.Temp);
            var set = new HashSet<ComponentType>(); foreach (var component in components) set.Add(component);
            if (!new[] { ComponentType.ReadWrite<AirplaneStop>(), ComponentType.ReadWrite<Game.Routes.TakeoffLocation>(), ComponentType.ReadWrite<Game.Objects.OutsideConnection>(), ComponentType.ReadWrite<RandomLocalizationIndex>(), ComponentType.ReadWrite<Transform>(), ComponentType.ReadWrite<PrefabRef>() }.All(set.Contains)) continue;
            prefabEntity = entity; objectData = data; selectedPrefabName=prefab.name; log.Diagnostic($"AIR prefab={prefab.name}; transfer=Air; stop=Airplane; requiredArchetypeComponentsVerified=true"); break;
        }
        if (prefabEntity == Entity.Null) return "Air connections skipped: no verified native airplane outside-connection object prefab.";
        var placed = AirPlanner.Place(prepared.AirDestinations, half, Math.Max(a.y, b.y));
        log.Count("Air perimeter candidates", placed.Count);
        log.Count("Air perimeter spacing rejected", prepared.AirDestinations.Count - placed.Count);
        foreach(var requested in prepared.AirDestinations)
            log.Diagnostic($"AIR REQUEST name={requested.Name}; distanceKm={requested.DistanceKm:0.###}; bearingDegrees={requested.Bearing:0.###}; perimeterCandidate={placed.Any(p=>p.Name==requested.Name&&p.Bearing==requested.Bearing)}");
        using var query = manager.CreateEntityQuery(new EntityQueryDesc { All = new[] { ComponentType.ReadOnly<AirplaneStop>(), ComponentType.ReadOnly<Game.Objects.OutsideConnection>(), ComponentType.ReadOnly<Transform>() }, None = new[] { ComponentType.ReadOnly<Deleted>(), ComponentType.ReadOnly<Game.Tools.Temp>() } });
        using var existing = query.ToComponentDataArray<Transform>(Allocator.Temp);
        var positions = new List<float3>(); foreach (var transform in existing) positions.Add(transform.m_Position);
        var created = 0; var skipped = prepared.AirDestinations.Count - placed.Count;
        foreach (var destination in placed)
        {
            var entity = Entity.Null;
            try
            {
                if (positions.Any(p => math.distance(p.xz, destination.Position.xz) < 200)) { skipped++; log.Diagnostic($"AIR SKIP name={destination.Name}; reason=existing-connection-spacing; position={destination.Position}"); continue; }
                var terrainHeight = NetworkProcessor.Sample(destination.Position.x, destination.Position.z, prepared.Heightmap, prepared.HeightScaleMeters, prepared.Request.MapSize.WidthMeters);
                if (destination.Position.y < terrainHeight + 100) { skipped++; log.Warn($"Air destination {destination.Name} skipped: native airway altitude intersects high terrain."); continue; }
                // Verified GenerateObjectsSystem uses ObjectData.m_Archetype + Transform + PrefabRef.
                // Native Created/Updated initialization resolves stop, takeoff and outside connection data.
                entity = manager.CreateEntity(objectData.m_Archetype);
                manager.SetComponentData(entity, new PrefabRef { m_Prefab = prefabEntity });
                manager.SetComponentData(entity, new Transform(destination.Position, quaternion.identity));
                if (!manager.HasComponent<Created>(entity)) manager.AddComponentData(entity, default(Created));
                if (!manager.HasComponent<Updated>(entity)) manager.AddComponentData(entity, default(Updated));
                names.SetCustomName(entity, destination.Name);
                try
                {
                    var actual=manager.GetComponentData<Transform>(entity).m_Position;
                    log.Diagnostic($"AIR READBACK name={destination.Name}; requestedPosition={destination.Position}; actualPosition={actual}; positionMatch={math.distance(actual,destination.Position)<.01f}; prefabMatch={manager.GetComponentData<PrefabRef>(entity).m_Prefab==prefabEntity}; airplaneStop={manager.HasComponent<AirplaneStop>(entity)}; outsideConnection={manager.HasComponent<Game.Objects.OutsideConnection>(entity)}; takeoffLocation={manager.HasComponent<Game.Routes.TakeoffLocation>(entity)}; initializationStillPending=true");
                }
                catch(Exception diagnosticException){log.Diagnostic($"AIR READBACK unavailable name={destination.Name}: {diagnosticException}");}
                positions.Add(destination.Position); created++;
                NativeDiagnostics.RememberAir(destination.Name,selectedPrefabName,destination.Position);
            }
            catch (Exception exception)
            {
                if (entity != Entity.Null) manager.DestroyEntity(entity);
                skipped++; log.Warn($"Air destination {destination.Name} skipped: {exception.Message}"); log.Diagnostic($"AIR FAILURE name={destination.Name}: {exception}");
            }
        }
        var report = $"Air connections created: {created}; skipped: {skipped}; native perimeter {half * 2 / 1000:0.###} km; minimum spacing 200 m. Native route initialization runs on the next game update.";
        NativeDiagnostics.AirApplied=created>0;
        log.Count("Air connections created", created);
        log.Count("Air connections skipped", skipped);
        log.Diagnostic($"AIR TOTAL requested={prepared.AirDestinations.Count}; perimeterCandidates={placed.Count}; created={created}; skipped={skipped}; functionalFlightsVerified=false");
        log.Info(report); return report;
    }
}
