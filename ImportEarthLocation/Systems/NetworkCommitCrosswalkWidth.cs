using System;
using Colossal.Mathematics;
using Game.Net;
using Game.Tools;
using Unity.Collections;
using Unity.Entities;

namespace ImportEarthLocation.Systems
{
    // Interoperability with the CrosswalkWidth mod (Paradox Mods 158875), read from its 11-2026 build.
    //
    // Apply marks the original of every temporary lane Updated (ApplyNetSystem.Update). In the same
    // frame LaneSystem rebuilds the junction and, for a lane it keeps, queues SetBuffer<CutRange>
    // because the lane has that buffer when its job runs. CrosswalkWidth's CrosswalkOverrideSystem
    // updates right after LaneSystem, before that queue plays back. It treats an Updated crossing
    // lane whose CutRange buffer is one range covering the whole lane as paint it has hidden, and
    // removes the buffer at once. The queued SetBuffer then plays back on a lane without the buffer
    // and the game exits inside Unity. Very short crossings, common in imported junctions, get that
    // whole-lane cut from the game itself.
    //
    // Writing the same cut as two halves makes CrosswalkWidth leave the buffer alone. LaneSystem's
    // queued write replaces the buffer in that frame, so the halves do not outlive the apply.
    public sealed partial class NetworkCommitSystem
    {
        private const string CrosswalkWidthAssembly = "CrosswalkWidth";
        private const string CrosswalkWidthSystem = "CrosswalkWidth.Systems.CrosswalkOverrideSystem";

        // The limits CrosswalkWidth uses to recognise a whole-lane cut.
        private const float WholeLaneCutStart = 0.0001f;
        private const float WholeLaneCutEnd = 0.9999f;

        private EntityQuery _temporaryLaneQuery;
        private bool? _crosswalkWidthActive;

        private bool CrosswalkWidthActive()
        {
            // Mods finish loading before a map can be imported, so the first apply can decide for the session.
            if (_crosswalkWidthActive.HasValue)
                return _crosswalkWidthActive.Value;

            var active = false;

            foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (assembly.GetName().Name != CrosswalkWidthAssembly)
                    continue;

                var systemType = assembly.GetType(CrosswalkWidthSystem, throwOnError: false);
                active = systemType != null && World.GetExistingSystemManaged(systemType) != null;

                break;
            }

            _crosswalkWidthActive = active;

            return active;
        }

        private void SplitWholeLaneCutsForCrosswalkWidth()
        {
            if (!CrosswalkWidthActive())
                return;

            if (_temporaryLaneQuery == default)
                _temporaryLaneQuery = GetEntityQuery(ComponentType.ReadOnly<Temp>(), ComponentType.ReadOnly<Lane>());

            using var temporaryLanes = _temporaryLaneQuery.ToComponentDataArray<Temp>(Allocator.Temp);
            var splitCuts = 0;

            foreach (var temporaryLane in temporaryLanes)
            {
                var original = temporaryLane.m_Original;

                if (original == Entity.Null || !EntityManager.HasBuffer<CutRange>(original))
                    continue;

                var cuts = EntityManager.GetBuffer<CutRange>(original);

                if (cuts.Length != 1)
                    continue;

                var cut = cuts[0].m_CurveDelta;

                if (cut.min > WholeLaneCutStart || cut.max < WholeLaneCutEnd)
                    continue;

                cuts[0] = new CutRange { m_CurveDelta = new Bounds1(cut.min, 0.5f) };
                cuts.Add(new CutRange { m_CurveDelta = new Bounds1(0.5f, cut.max) });
                splitCuts++;
            }

            if (splitCuts != 0)
                s_Log?.Diagnostic($"NATIVE COMMIT CrosswalkWidth guard split wholeLaneCuts={splitCuts}");
        }
    }
}
