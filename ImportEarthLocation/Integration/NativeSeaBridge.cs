using System;
using System.Collections.Generic;
using Game.Net;
using Game.Prefabs;
using Game.Common;
using Game.Tools;
using Game.Simulation;
using Colossal.Mathematics;
using Unity.Mathematics;
using ImportEarthLocation.Core;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Integration;

/// <summary>Stock waterway definitions; native systems create lanes and outside objects.</summary>
public static class NativeSeaBridge
{
    private static readonly List<float3> ExpectedEndpoints=new();
    public static void Clear()=>ExpectedEndpoints.Clear();
    public static IReadOnlyList<Entity> CreateDefinitions(PreparedImport prepared,EntityManager manager,PrefabSystem prefabs,TerrainSystem terrain,ImportLog log)
    {
        Clear();
        var result=new List<Entity>();var ocean=prepared.Ocean;
        if(ocean==null){log.Warn("Sea connections: directed ocean coverage unavailable.");return result;}
        Entity selected=Entity.Null;
        foreach(var prefab in NativePrefabCatalog.Read<WaterwayPrefab>(manager,prefabs))
        {
            var entity=prefabs.GetEntity(prefab);var ship=false;var lane=false;
            if(manager.HasBuffer<Game.Prefabs.SubObject>(entity))foreach(var sub in manager.GetBuffer<Game.Prefabs.SubObject>(entity,true))
                if(((int)sub.m_Flags&32)!=0&&(((int)sub.m_Flags&4096)==0||((int)sub.m_Flags&(131072|262144))!=0)&&manager.HasComponent<OutsideConnectionData>(sub.m_Prefab)&&
                    (manager.GetComponentData<OutsideConnectionData>(sub.m_Prefab).m_Type&OutsideConnectionTransferType.Ship)!=0)ship=true;
            if(manager.HasBuffer<DefaultNetLane>(entity))foreach(var item in manager.GetBuffer<DefaultNetLane>(entity,true))
                if((item.m_Flags&LaneFlags.Road)!=0&&manager.HasComponent<CarLaneData>(item.m_Lane)&&(manager.GetComponentData<CarLaneData>(item.m_Lane).m_RoadTypes&RoadTypes.Watercraft)!=0)lane=true;
            log.Diagnostic($"SEA PREFAB name={prefab.name}; shipOutsideSubObject={ship}; initializedWatercraftLane={lane}");
            if(ship&&lane&&manager.HasComponent<NetGeometryData>(entity)&&(manager.GetComponentData<NetGeometryData>(entity).m_Flags&GeometryFlags.OnWater)!=0){selected=entity;break;}
        }
        if(selected==Entity.Null){log.Warn("Sea connections: stock ship outside-object/watercraft lane contract unavailable.");return result;}
        if(!NativeMapBoundary.TryGetHalf(terrain,prepared.Request.MapSize.WidthMeters,prepared.Heightmap.Length,log,out var half))
        {log.Warn("Sea connections: native terrain registration unavailable.");return result;}
        var n=ocean.Resolution;
        var heights=terrain.GetHeightData(false);var bounds=TerrainUtils.GetBounds(ref heights);
        // Height-data bounds describe sample centres, not nominal outer corners.
        // Permit one raster cell of registration difference, not arbitrary maps.
        var terrainN=(int)Math.Sqrt(prepared.Heightmap.Length);
        var tolerance=Math.Max(2,2*half/Math.Max(1,terrainN-1));
        log.Diagnostic($"SEA BOUNDS nativeMin={bounds.min}; nativeMax={bounds.max}; preparedHalf={half}; registrationToleranceMeters={tolerance}; oceanOnly=true");
        if(math.abs(bounds.min.x+half)>tolerance||math.abs(bounds.max.x-half)>tolerance||math.abs(bounds.min.z+half)>tolerance||math.abs(bounds.max.z-half)>tolerance)
        {log.Warn("Sea connections: native generation perimeter differs from prepared map bounds; no invented outside marker created.");return result;}
        var rejectedCoverage=0;var rejectedDepth=0;var rejectedBounds=0;
        bool Wet(float3 p)
        {
            var x=(int)Math.Round((p.x/ (2*half)+.5f)*(n-1));var z=(int)Math.Round((p.z/(2*half)+.5f)*(n-1));
            if(x<0||z<0||x>=n||z>=n){rejectedBounds++;return false;}
            if(ocean.Classification[z*n+x]!=1){rejectedCoverage++;return false;}
            var tx=math.clamp((int)Math.Round((p.x/(2*half)+.5f)*(terrainN-1)),0,terrainN-1);
            var tz=math.clamp((int)Math.Round((p.z/(2*half)+.5f)*(terrainN-1)),0,terrainN-1);
            var deep=prepared.Heightmap[tz*terrainN+tx]*prepared.HeightScaleMeters/65535f<prepared.SeaLevelMeters-2;
            if(!deep)rejectedDepth++;return deep;
        }
        // Stratified deterministic variation: count scales with wet external perimeter,
        // not city names. Paths remain short, straight and separated from corners.
        var eligible=0;var candidates=new List<(float3 a,float3 b)>();
        for(var side=0;side<4;side++)for(var slot=0;slot<8;slot++)
        {
            var jitter=(math.hash(new int3(side,slot,(int)(prepared.Request.Longitude*1000)))%1000)/1000f;
            var along=math.lerp(-half*.9f,half*.9f,(slot+.25f+.5f*jitter)/8);
            var outward=side==0?new float3(-1,0,0):side==1?new float3(1,0,0):side==2?new float3(0,0,-1):new float3(0,0,1);
            var edge=side==0||side==2?-half:half;
            var a=side<2?new float3(edge+outward.x*.25f,prepared.SeaLevelMeters,along):new float3(along,prepared.SeaLevelMeters,edge+outward.z*.25f);
            var b=a-outward*math.min(600,half*.1f);var fits=true;
            for(var i=0;i<=12;i++)if(!Wet(math.lerp(a-outward*.5f,b,i/12f))){fits=false;break;}
            if(fits){eligible++;candidates.Add((a,b));}
        }
        for(var i=0;i<candidates.Count;i++)
        {
            // At most eight connections, stratified across eligible wet edge slots.
            if(i%Math.Max(1,(int)Math.Ceiling(candidates.Count/8d))!=0)continue;
            var (a,b)=candidates[i];var direction=math.normalizesafe(b-a);var rotation=quaternion.LookRotationSafe(direction,new float3(0,1,0));
            var curve=new Bezier4x3(a,math.lerp(a,b,1/3f),math.lerp(a,b,2/3f),b);var definition=manager.CreateEntity();
            try
            {
                CoursePos Position(float3 p,bool first)=>new(){m_Position=p,m_Rotation=rotation,m_CourseDelta=first?0:1,m_ParentMesh=-1,
                    m_Flags=(first?CoursePosFlags.IsFirst:CoursePosFlags.IsLast)|CoursePosFlags.IsLeft|CoursePosFlags.IsRight|CoursePosFlags.DisableMerge};
                manager.AddComponentData(definition,new CreationDefinition{m_Prefab=selected,m_RandomSeed=i+1});
                manager.AddComponentData(definition,new NetCourse{m_Curve=curve,m_StartPosition=Position(a,true),m_EndPosition=Position(b,false),m_Length=math.distance(a,b),m_FixedIndex=-1});
                manager.AddComponentData(definition,default(Updated));result.Add(definition);ExpectedEndpoints.Add(a);
                log.Diagnostic($"SEA COURSE start={a}; end={b}; straight=true; outsideEndpoint=true; predictedDepthAtLeastMeters=2; simulationDepthUnverified=true");
            }
            catch(Exception error){manager.DestroyEntity(definition);log.Warn("Sea course omitted locally: "+error.Message);}
        }
        log.Count("Sea courses submitted",result.Count);log.Diagnostic($"SEA PLAN eligibleWetPerimeterSlots={eligible}/32; definitions={result.Count}; rejectedCoverage={rejectedCoverage}; rejectedDepth={rejectedDepth}; rejectedBounds={rejectedBounds}; nativeOutsideObjectsRequired=true; riverConnectionsExcluded=true");
        return result;
    }
    public static string Assess(PreparedImport prepared, EntityManager manager, PrefabSystem prefabs, ImportLog log)
    {
        var records=new List<object>();var stockEdges=0;
        using(var query=manager.CreateEntityQuery(ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()))
        using(var entities=query.ToEntityArray(Allocator.Temp))
        {
            foreach(var entity in entities)
            {
                var reference=manager.GetComponentData<PrefabRef>(entity);
                if(!prefabs.TryGetPrefab<WaterwayPrefab>(reference.m_Prefab,out var prefab)||prefab==null)continue;
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
        return $"Sea assessment saved: {stockEdges} native waterway edges observed. Outside-object initialization and shipping traffic still require in-game verification.";
    }

    public static void Readback(EntityManager manager,ImportLog log)
    {
        if(ExpectedEndpoints.Count==0)return;
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{
            All=new[]{ComponentType.ReadOnly<Game.Objects.OutsideConnection>(),ComponentType.ReadOnly<Game.Objects.Transform>(),ComponentType.ReadOnly<PrefabRef>()},
            None=new[]{ComponentType.ReadOnly<Deleted>(),ComponentType.ReadOnly<Temp>()}});
        using var entities=query.ToEntityArray(Allocator.Temp);
        var actual=new List<float3>();var pending=0;
        foreach(var entity in entities)
        {
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;
            if(!manager.HasComponent<OutsideConnectionData>(prefab)||(manager.GetComponentData<OutsideConnectionData>(prefab).m_Type&OutsideConnectionTransferType.Ship)==0)continue;
            var position=manager.GetComponentData<Game.Objects.Transform>(entity).m_Position;actual.Add(position);
            if(manager.HasComponent<Created>(entity))pending++;
            if(actual.Count<=32)log.Diagnostic($"SEA OUTSIDE OBJECT entity={entity}; position={position}; createdMarkerStillPresent={manager.HasComponent<Created>(entity)}; shipPrefabVerified=true; functionalTrafficVerified=false");
        }
        var matched=0;foreach(var expected in ExpectedEndpoints)if(actual.Exists(p=>math.distance(p.xz,expected.xz)<=64))matched++;
        log.Diagnostic($"SEA OUTSIDE READBACK submittedEndpoints={ExpectedEndpoints.Count}; matchingNativeShipObjects={matched}; nativeShipObjects={actual.Count}; pendingInitialization={pending}; outsideObjectsMissing={ExpectedEndpoints.Count-matched}; trafficUnverified=true; matchingToleranceMeters=64");
        if(matched<ExpectedEndpoints.Count)log.Warn($"Sea connections: {ExpectedEndpoints.Count-matched} submitted endpoints have no observed ship outside object yet; see SEA OUTSIDE READBACK.");
    }
}
