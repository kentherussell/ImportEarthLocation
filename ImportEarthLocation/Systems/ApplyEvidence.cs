using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using Unity.Collections;
using Unity.Entities;
using SubLane = Game.Net.SubLane;

namespace ImportEarthLocation.Systems
{
    // What the game is about to apply, saved before each apply so that it survives a game
    // exit during that apply. The files are written and closed at once; the run archive
    // writes in the background and loses the last batch when the process dies.
    //
    // type-indices.txt lists every component type index of this game session, so the type
    // lists found in a crash dump can be named. last-apply.txt holds the preview entities of
    // the latest batch only: their archetypes, the existing entities they will change, where
    // they are and whether their lane lists are sound. previous-apply.txt is the batch before
    // it, and apply-history.txt has one line for every batch.
    internal static class ApplyEvidence
    {
        private static string? s_TypesWrittenFor;

        public static void Write(EntityManager manager, string folder, string? batch)
        {
            var target = Path.Combine(folder, "crash-evidence");
            Directory.CreateDirectory(target);

            if (s_TypesWrittenFor != folder)
            {
                WriteTypes(Path.Combine(target, "type-indices.txt"));
                s_TypesWrittenFor = folder;
            }

            var text = new StringBuilder();
            var archetypes = new Dictionary<EntityArchetype, int>();
            var archetypeLines = new List<string>();

            int ArchetypeId(Entity entity)
            {
                var archetype = manager.GetChunk(entity).Archetype;

                if (archetypes.TryGetValue(archetype, out var id))
                    return id;

                id = archetypes.Count;
                archetypes[archetype] = id;

                using var types = archetype.GetComponentTypes(Allocator.Temp);
                var line = new StringBuilder("archetype " + id + ":");

                foreach (var type in types)
                    line.Append(' ').Append(type.TypeIndex.Value.ToString("X"));

                archetypeLines.Add(line.ToString());

                return id;
            }

            using var query = manager.CreateEntityQuery(ComponentType.ReadOnly<Temp>());
            using var entities = query.ToEntityArray(Allocator.Temp);

            text.Append("batch ").Append(batch).Append("; preview entities ").Append(entities.Length).Append('\n');
            text.Append("entity archetype flags original originalArchetype (-1: the original does not exist)\n");

            foreach (var entity in entities)
            {
                var temporary = manager.GetComponentData<Temp>(entity);
                var original = temporary.m_Original;
                var originalArchetype = manager.Exists(original) ? ArchetypeId(original) : -1;

                text.Append(entity.Index).Append(':').Append(entity.Version).Append(' ');
                text.Append(ArchetypeId(entity)).Append(' ');
                text.Append(((uint)temporary.m_Flags).ToString("X")).Append(' ');
                text.Append(original.Index).Append(':').Append(original.Version).Append(' ');
                text.Append(originalArchetype).Append('\n');
            }

            foreach (var line in archetypeLines)
                text.Append(line).Append('\n');

            var oddLanes = AppendLaneLists(manager, entities, text);

            // The batch before the latest stays on disk, so a batch that ends the game can be
            // compared with the good one just before it.
            var latest = Path.Combine(target, "last-apply.txt");
            var previous = Path.Combine(target, "previous-apply.txt");

            if (File.Exists(latest))
            {
                File.Delete(previous);
                File.Move(latest, previous);
            }

            File.WriteAllText(latest, text.ToString());

            File.AppendAllText(
                Path.Combine(target, "apply-history.txt"),
                batch + " preview entities " + entities.Length + " odd lanes " + oddLanes + "\n");
        }

        // Where each preview entity is, and whether the lane lists of the previewed nodes and
        // edges (and of the existing ones they change) are sound. A lane is odd when it no
        // longer exists, belongs to another owner, or is listed more than once.
        private static int AppendLaneLists(EntityManager manager, NativeArray<Entity> entities, StringBuilder text)
        {
            text.Append("entity owner prefab curveStart curveEnd (metres; - where the entity has none)\n");

            foreach (var entity in entities)
            {
                text.Append(entity.Index).Append(':').Append(entity.Version).Append(' ');

                if (manager.HasComponent<Owner>(entity))
                {
                    var owner = manager.GetComponentData<Owner>(entity).m_Owner;
                    text.Append(owner.Index).Append(':').Append(owner.Version);

                    if (!manager.Exists(owner))
                        text.Append("(missing)");
                }
                else
                {
                    text.Append('-');
                }

                text.Append(' ');

                if (manager.HasComponent<PrefabRef>(entity))
                    text.Append(manager.GetComponentData<PrefabRef>(entity).m_Prefab.Index);
                else
                    text.Append('-');

                if (manager.HasComponent<Curve>(entity))
                {
                    var curve = manager.GetComponentData<Curve>(entity).m_Bezier;
                    text.Append(' ').Append(curve.a.x.ToString("F1")).Append(',').Append(curve.a.z.ToString("F1"));
                    text.Append(' ').Append(curve.d.x.ToString("F1")).Append(',').Append(curve.d.z.ToString("F1"));
                }
                else
                {
                    text.Append(" - -");
                }

                text.Append('\n');
            }

            var listedBy = new Dictionary<Entity, Entity>();
            var checkedOwners = new HashSet<Entity>();
            var odd = 0;

            void CheckLanes(Entity owner)
            {
                if (!checkedOwners.Add(owner) || !manager.Exists(owner) || !manager.HasBuffer<SubLane>(owner))
                    return;

                var lanes = manager.GetBuffer<SubLane>(owner, true);

                for (var i = 0; i < lanes.Length; i++)
                {
                    var lane = lanes[i].m_SubLane;
                    string? problem = null;

                    if (!manager.Exists(lane))
                        problem = "does not exist";
                    else if (!manager.HasComponent<Owner>(lane))
                        problem = "has no owner";
                    else if (manager.GetComponentData<Owner>(lane).m_Owner != owner)
                        problem = "is owned by " + manager.GetComponentData<Owner>(lane).m_Owner.Index;
                    else if (listedBy.TryGetValue(lane, out var other))
                        problem = "is also listed by " + other.Index;

                    listedBy[lane] = owner;

                    if (problem == null)
                        continue;

                    odd++;
                    text.Append("odd lane ").Append(lane.Index).Append(':').Append(lane.Version);
                    text.Append(" listed by ").Append(owner.Index).Append(':').Append(owner.Version);
                    text.Append(' ').Append(problem).Append('\n');
                }
            }

            foreach (var entity in entities)
            {
                CheckLanes(entity);
                CheckLanes(manager.GetComponentData<Temp>(entity).m_Original);
            }

            text.Append("odd lanes ").Append(odd).Append('\n');

            return odd;
        }

        private static void WriteTypes(string path)
        {
            var text = new StringBuilder("type index with flags (hex), type\n");

            foreach (var type in TypeManager.GetAllTypes())
            {
                if (type.Type == null)
                    continue;

                text.Append(type.TypeIndex.Value.ToString("X")).Append(' ').Append(type.Type.FullName).Append('\n');
            }

            File.WriteAllText(path, text.ToString());
        }
    }
}
