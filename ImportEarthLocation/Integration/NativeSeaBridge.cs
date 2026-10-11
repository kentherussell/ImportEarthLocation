using System;
using System.Collections.Generic;
using Game.Net;
using Game.Prefabs;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

/// <summary>Read-only stock shipping contract capture. Never substitutes air or road creation.</summary>
public static class NativeSeaBridge
{
    public static string Assess(PreparedImport prepared, EntityManager manager, PrefabSystem prefabs, ImportLog log)
    {
        var records=new List<object>();var stockEdges=0;
        using(var query=manager.CreateEntityQuery(ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()))
        using(var entities=query.ToEntityArray(Allocator.Temp))
        {
            foreach(var entity in entities)
            {
                var reference=manager.GetComponentData<PrefabRef>(entity);
                if(!prefabs.TryGetPrefab<WaterwayPrefab>(reference.m_Prefab,out var prefab))continue;
                stockEdges++;
                if(records.Count>=128)continue;
                var edge=manager.GetComponentData<Edge>(entity);var curve=manager.GetComponentData<Curve>(entity).m_Bezier;
                var components=new List<string>();
                using(var types=manager.GetComponentTypes(entity,Allocator.Temp))foreach(var type in types)components.Add(type.GetManagedType()?.FullName??type.ToString());
                object Point(Unity.Mathematics.float3 p)=>new {x=p.x,y=p.y,z=p.z};
                records.Add(new {entity=entity.Index,version=entity.Version,prefab=prefab.name,start=edge.m_Start.Index,end=edge.m_End.Index,a=Point(curve.a),b=Point(curve.b),c=Point(curve.c),d=Point(curve.d),components});
            }
        }
        log.DiagnosticRecords("sea-stock-reference",records);
        var ocean=prepared.Ocean;var accepted=0;var ambiguous=0;var unseeded=0;
        if(ocean!=null)foreach(var label in ocean.Classification){if(label==1)accepted++;else if(label==2)ambiguous++;else if(label==3)unseeded++;}
        log.Diagnostic($"SEA CONTRACT stockWaterwayEdges={stockEdges}; captured={records.Count}; oceanCells={accepted}; ambiguousCells={ambiguous}; unseededCells={unseeded}; coastVersion={ocean?.Version??"unavailable"}; engineMvid={typeof(Edge).Assembly.ManifestModule.ModuleVersionId}; mutation=false; shippingInitializationVerified=false; simulationDepthVerified=false");
        log.Count("Sea stock reference edges",stockEdges);
        return "Sea assessment saved. Shipping creation deferred: stock outside-connection initialization and simulated navigable depth are not verified. No shipping entities changed.";
    }
}
