using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using SubLane=Game.Net.SubLane;
using CarLane=Game.Net.CarLane;
using TrackLane=Game.Net.TrackLane;

namespace ImportEarthLocation.Integration;

// Diagnostics only. Content/incident hashes survive entity renumbering; they
// compare persistence, not geometric correctness or arbitrary graph isomorphism.
public static class NativeNetworkPersistence
{
    public static IEnumerable<int> Capture(EntityManager manager,PrefabSystem prefabs,Action<JObject> publish,Action<string>? progress=null)
    {
        JObject? previous=null;
        for(var pass=0;pass<2;pass++)
        {
            JObject? snapshot=null;
            foreach(var slice in Read(manager,prefabs,value=>snapshot=value,pass+1,progress))yield return slice;
            if(snapshot==null)yield break;
            if(pass==0){previous=snapshot;yield return 0;continue;}
            var stable=JToken.DeepEquals(previous,snapshot);snapshot["stableAcrossTwoPasses"]=stable;
            snapshot["entityIdsCompared"]=false;snapshot["geometricAcceptanceProven"]=false;
            publish(snapshot);
        }
    }
    private static IEnumerable<int> Read(EntityManager manager,PrefabSystem prefabs,Action<JObject> publish,int pass,Action<string>? progress)
    {
        var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=0d;
        void Progress(string phase,int completed,int total)
        {
            if(progress==null||clock.Elapsed.TotalSeconds<nextProgress)return;
            nextProgress=clock.Elapsed.TotalSeconds+5;
            progress($"NATIVE PERSISTENCE pass={pass}/2; phase={phase}; completed={completed}; total={total}; elapsedSeconds={clock.Elapsed.TotalSeconds:F1}; distributedAcrossFrames=true");
        }
        manager.CompleteAllTrackedJobs();
        using var query=manager.CreateEntityQuery(new EntityQueryDesc{All=new[]{ComponentType.ReadOnly<Edge>(),ComponentType.ReadOnly<Curve>(),ComponentType.ReadOnly<PrefabRef>()},None=new[]{ComponentType.ReadOnly<Temp>(),ComponentType.ReadOnly<Deleted>()}});
        // Temp allocations cannot cross yielded frame boundaries.
        using var entities=query.ToEntityArray(Allocator.Persistent);
        var edges=new Dictionary<Entity,string>();var incidence=new Dictionary<Entity,List<string>>();var edgeRecords=new List<string>();var missing=0;
        string Number(float x)=>x.ToString("R",CultureInfo.InvariantCulture);
        string Point(float3 p)=>Number(p.x)+","+Number(p.y)+","+Number(p.z);
        string Asset(Entity entity)
        {
            if(!manager.HasComponent<PrefabRef>(entity))return "unavailable";
            var prefab=manager.GetComponentData<PrefabRef>(entity).m_Prefab;
            if(!prefabs.TryGetPrefab<PrefabBase>(prefab,out var resolved)||resolved==null){missing++;return "unavailable";}
            return resolved.name;
        }
        string CurveRecord(Colossal.Mathematics.Bezier4x3 c)=>Point(c.a)+";"+Point(c.b)+";"+Point(c.c)+";"+Point(c.d);
        for(var i=0;i<entities.Length;i++)
        {
            if((i&127)==0){Progress("edges",i,entities.Length);yield return 0;manager.CompleteAllTrackedJobs();}
            var e=entities[i];if(!manager.HasComponent<Edge>(e)||!manager.HasComponent<Curve>(e)||manager.HasComponent<Deleted>(e)){missing++;continue;}
            var prefab=manager.GetComponentData<PrefabRef>(e).m_Prefab;
            if(!manager.HasComponent<RoadData>(prefab)&&!manager.HasComponent<TrackData>(prefab))continue;
            var edge=manager.GetComponentData<Edge>(e);
            if(!manager.HasComponent<Node>(edge.m_Start)||!manager.HasComponent<Node>(edge.m_End)){missing++;continue;}
            var composition=manager.HasComponent<Composition>(e)?manager.GetComponentData<Composition>(e).m_Edge:Entity.Null;
            var parts=new List<string>{Asset(e),CurveRecord(manager.GetComponentData<Curve>(e).m_Bezier)};
            if(manager.HasComponent<NetCompositionData>(composition))
            {
                var data=manager.GetComponentData<NetCompositionData>(composition);
                parts.Add($"composition:{data.m_Flags.m_General}:{data.m_Flags.m_Left}:{data.m_Flags.m_Right}:{Number(data.m_Width)}:{Number(data.m_HeightRange.min)}:{Number(data.m_HeightRange.max)}");
                if(manager.HasBuffer<NetCompositionLane>(composition))foreach(var lane in manager.GetBuffer<NetCompositionLane>(composition,true))
                    parts.Add($"slot:{lane.m_Index}:{lane.m_Flags}:{Point(lane.m_Position)}");
                else missing++;
            }
            else missing++;
            var identity=NetworkPersistenceHash.Digest(parts);edges[e]=identity;edgeRecords.Add(identity);
            foreach(var endpoint in new[]{(edge.m_Start,"start"),(edge.m_End,"end")})
            {if(!incidence.TryGetValue(endpoint.Item1,out var list))incidence[endpoint.Item1]=list=new();list.Add(identity+":"+endpoint.Item2);}
        }
        var nodes=new Dictionary<Entity,string>();var nodeRecords=new List<string>();var processed=0;
        foreach(var pair in incidence)
        {
            if((processed++&127)==0){Progress("nodes",processed-1,incidence.Count);yield return 0;manager.CompleteAllTrackedJobs();}
            if(!manager.HasComponent<Node>(pair.Key)){missing++;continue;}
            var hash=NetworkPersistenceHash.Digest(pair.Value.Concat(new[]{Point(manager.GetComponentData<Node>(pair.Key).m_Position)}));
            nodes[pair.Key]=hash;nodeRecords.Add(hash);
        }
        var owners=edges.Keys.Concat(nodes.Keys).ToArray();var edgeByIndex=edges.ToDictionary(p=>p.Key.Index,p=>p.Value);var nodeByIndex=nodes.ToDictionary(p=>p.Key.Index,p=>p.Value);
        string Path(Game.Pathfind.PathNode path)
        {
            var index=path.GetOwnerIndex();
            if(edgeByIndex.TryGetValue(index,out var edge))return "edge:"+edge+":"+path.GetLaneIndex()+":"+Number(path.GetCurvePos());
            if(nodeByIndex.TryGetValue(index,out var node))return "node:"+node+":"+path.GetLaneIndex()+":"+Number(path.GetCurvePos());
            missing++;return "unresolved-owner";
        }
        var ownerRecords=new List<string>();long lanes=0;
        for(var i=0;i<owners.Length;i++)
        {
            if((i&63)==0){Progress("lane-owners",i,owners.Length);yield return 0;manager.CompleteAllTrackedJobs();}
            var owner=owners[i];var records=new List<string>();
            if(manager.HasBuffer<SubLane>(owner))foreach(var sub in manager.GetBuffer<SubLane>(owner,true))
            {
                var entity=sub.m_SubLane;if(!manager.HasComponent<Lane>(entity)||manager.HasComponent<Deleted>(entity)||manager.HasComponent<Temp>(entity))continue;
                if(!manager.HasComponent<Owner>(entity)||manager.GetComponentData<Owner>(entity).m_Owner!=owner)continue;
                var car=manager.HasComponent<CarLane>(entity);var rail=manager.HasComponent<TrackLane>(entity);if(!car&&!rail)continue;
                var lane=manager.GetComponentData<Lane>(entity);
                var record=(car?"car:":"track:")+Path(lane.m_StartNode)+">"+Path(lane.m_EndNode);
                record+=car?":"+manager.GetComponentData<CarLane>(entity).m_Flags:":"+manager.GetComponentData<TrackLane>(entity).m_Flags;
                if(manager.HasComponent<Curve>(entity))record+=";"+CurveRecord(manager.GetComponentData<Curve>(entity).m_Bezier);else missing++;
                records.Add(record);lanes++;
            }
            else missing++;
            ownerRecords.Add(NetworkPersistenceHash.Digest(records.Concat(new[]{edges.TryGetValue(owner,out var edgeIdentity)?"edge-owner:"+edgeIdentity:"node-owner:"+nodes[owner]})));
        }
        if(query.CalculateEntityCount()!=entities.Length)missing++;
        publish(new JObject{["schema"]=1,["edges"]=edges.Count,["nodes"]=nodes.Count,["lanes"]=lanes,["unavailable"]=missing,
            ["edgeContentHash"]=NetworkPersistenceHash.Digest(edgeRecords),["nodeIncidenceHash"]=NetworkPersistenceHash.Digest(nodeRecords),["laneMovementHash"]=NetworkPersistenceHash.Digest(ownerRecords)});
    }
}
