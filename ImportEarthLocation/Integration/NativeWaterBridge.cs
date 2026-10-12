using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Game.Common;
using Game.Simulation;
using ImportEarthLocation.Core;
using Unity.Entities;
using Unity.Mathematics;
using Transform = Game.Objects.Transform;

namespace ImportEarthLocation.Integration;

public static class NativeWaterBridge
{
    public static string Apply(PreparedImport prepared, EntityManager manager, TerrainSystem terrain, WaterSystem water, ImportLog log)
    {
        var plans = prepared.Water?.Sources;
        log.Diagnostic($"WATER APPLY plans={plans?.Count??0}; loaded={water.Loaded}; actualCellSize={water.CellSize:R}; waterSimSpeed={water.WaterSimSpeed}; sourceMode=clean-target-depth");
        if (plans == null || plans.Count == 0) return "River water: no safe upstream sources planned; see WATER PLAN/skipped diagnostics. Existing sea plane retained.";
        var createdByBody=new Dictionary<long,int>();
        void ReportBodies(string reason)
        {
            foreach(var body in plans.GroupBy(p=>p.BodyId))
            {
                createdByBody.TryGetValue(body.Key,out var count);
                log.Diagnostic($"WATER APPLY BODY body={body.Key}; kind={body.First().BodyKind}; plannedControls={body.Count()}; createdControls={count}; missingControls={body.Count()-count}; noNativeControl={count==0}; reason={reason}; wetCoverageAccepted=false");
                if(count==0)log.Warn($"Water body {body.Key} has no safe native control after application. Its channel remains planned, but simulated water is not verified; inspect WATER APPLY SKIP diagnostics.");
            }
        }
        // The current clean-source GPU path uses target depth, not legacy constant-flow flags.
        var legacy = typeof(WaterSystem).GetProperty("UseLegacyWaterSources", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)?.GetValue(water);
        if (!water.Loaded || !math.isfinite(water.CellSize) || water.CellSize<=0 || !(legacy is bool mode) || mode)
        { ReportBodies("native-simulation-unavailable"); return "River water skipped: current non-legacy water simulation is unavailable; no guessed legacy flow settings used."; }
        manager.CompleteAllTrackedJobs();
        var heights = terrain.GetHeightData(false);
        if (!heights.heights.IsCreated) { ReportBodies("native-terrain-unavailable"); return "River water skipped: native terrain readback unavailable."; }
        var created = 0;
        foreach (var plan in plans)
        {
            var entity = Entity.Null;
            try
            {
                var position = plan.Position;
                position.y = TerrainUtils.SampleHeight(ref heights, position);
                var target=plan.TargetSurfaceLevel??plan.Position.y+plan.Depth;
                var nativeBank=plan.BankLevel;
                foreach(var bankProbe in plan.BankProbes)nativeBank=Math.Min(nativeBank,TerrainUtils.SampleHeight(ref heights,bankProbe));
                var unsafeReason=!math.isfinite(position.y)||!math.isfinite(target)||!math.isfinite(nativeBank)?"non-finite-native-height":
                    target>nativeBank-.25f?"stage-above-bank-budget":
                    plan.Radius<Math.Max(2,water.CellSize*.25f)?"below-local-source-size-policy":Math.Abs((prepared.Water?.WaterCellSize??0)-water.CellSize)>.01f?"native-grid-changed":null;
                if (unsafeReason!=null)
                { log.Warn($"River source {plan.WayId} skipped: native channel/bank or water grid differs from the safe plan.");log.Diagnostic($"WATER APPLY SKIP way={plan.WayId}; body={plan.BodyId}; nativeBed={position.y:R}; plannedBed={plan.Position.y:R}; target={target:R}; nativeBank={nativeBank:R}; radius={plan.Radius:R}; reason={unsafeReason}"); continue; }
                // Recheck the entire circular footprint against native terrain, including longitudinal slope.
                if(Math.Abs(position.y-plan.Position.y)>1.5f)
                    log.Diagnostic($"WATER BED RECOMPUTED body={plan.BodyId}; planned={plan.Position.y:R}; native={position.y:R}; targetUnchanged={target:R}; mismatchAloneDoesNotReject=true");
                bool Footprint(float radius)
                {
                    for(var ring=1;ring<=3;ring++)for(var i=0;i<16;i++)
                    {var angle=i*math.PI/8;var p=position+new float3(math.cos(angle),0,math.sin(angle))*(radius*ring/3);var sampled=TerrainUtils.SampleHeight(ref heights,p);if(!math.isfinite(sampled)||sampled>target-.1f)return false;}
                    return true;
                }
                var radius=plan.Radius;var minimumRadius=Math.Max(2,water.CellSize*.25f);var footprintSafe=Footprint(radius);
                while(!footprintSafe&&radius>minimumRadius){radius=Math.Max(minimumRadius,radius*.75f);footprintSafe=Footprint(radius);}
                if(footprintSafe&&radius<plan.Radius)log.Diagnostic($"WATER FOOTPRINT ADAPTED body={plan.BodyId}; originalRadius={plan.Radius:R}; acceptedRadius={radius:R}; targetLevel={target:R}; targetNotLowered=true; nativeTerrainVerified=true");
                if (!footprintSafe) { log.Warn($"Water body {plan.BodyId} source footprint reaches dry/higher native terrain.");log.Diagnostic($"WATER APPLY SKIP way={plan.WayId}; body={plan.BodyId}; position={position}; radius={plan.Radius}; depth={plan.Depth}; bankLevel={nativeBank}; reason=native-footprint-higher-than-stage; floodingBypass=false"); continue; }
                var nativeDepth=plan.TargetSurfaceLevel.HasValue?plan.TargetSurfaceLevel.Value-position.y:plan.Depth;
                if(!math.isfinite(nativeDepth)||nativeDepth<=0){log.Warn($"Water regulator {plan.WayId}: native bed is above its target surface; skipped this regulator, other water continues.");log.Diagnostic($"WATER APPLY SKIP way={plan.WayId}; body={plan.BodyId}; reason=native-bed-above-target; nativeDepth={nativeDepth:R}");continue;}
                var source = new WaterSourceData { m_ConstantDepth = 0, m_Radius = radius, m_Height = nativeDepth,
                    m_Multiplier = 1, m_Modifier = 1, m_Polluted = 0, m_Id = water.GetNextSourceId() };
                // Match persistent standalone editor water sources. No prefab initializer can overwrite depth.
                entity = manager.CreateEntity(ComponentType.ReadWrite<WaterSourceData>(), ComponentType.ReadWrite<Transform>(), ComponentType.ReadWrite<Created>(), ComponentType.ReadWrite<Updated>());
                manager.SetComponentData(entity, new Transform { m_Position = position, m_Rotation = quaternion.identity });
                manager.SetComponentData(entity, source);
                var actual = manager.GetComponentData<WaterSourceData>(entity);
                var actualPosition = manager.GetComponentData<Transform>(entity).m_Position;
                if (actual.m_Height != source.m_Height || actual.m_Radius != source.m_Radius || actual.m_Modifier != 1 || actual.m_Polluted != 0 || !math.all(actualPosition == position))
                    throw new InvalidOperationException("Native water-source component readback did not match.");
                NativeDiagnostics.RememberWater(actualPosition, actual, plan with{Radius=radius});
                created++;
                createdByBody.TryGetValue(plan.BodyId,out var previous);createdByBody[plan.BodyId]=previous+1;
                log.Diagnostic($"WATER CREATED way={plan.WayId}; name={plan.Name}; origin={plan.Origin}; entity={entity}; sourceId={actual.m_Id}; position={actualPosition}; radius={actual.m_Radius:R}; targetDepth={actual.m_Height:R}; targetLevel={actualPosition.y + actual.m_Height:R}; clean=true; modifier={actual.m_Modifier}; nativeComponentsReadBack=true; editorCreatedMarker=true; downstreamFillingVerified=false; fixedFlowRateNotUsed=true");
            }
            catch (Exception exception) when(exception is not OutOfMemoryException && exception is not OperationCanceledException)
            {
                if (entity != Entity.Null) manager.DestroyEntity(entity);
                log.Warn($"River source {plan.WayId} failed safely: {exception.GetBaseException().Message}");
                log.Diagnostic($"WATER FAILURE way={plan.WayId}: {exception}");
            }
        }
        NativeDiagnostics.WaterApplied = created > 0;
        ReportBodies("native-plan-revalidation");
        log.Count("River water sources created", created);
        return $"Water: {created}/{plans.Count} bank-limited river/lake sources created and read back. Wet coverage and containment require simulation verification.";
    }
}
