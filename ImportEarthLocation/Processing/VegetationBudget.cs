using System;
using System.Collections.Generic;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// How many of the planned trees and bushes an import keeps (report 121 §16.3).
//
// The density slider keeps the same share of every kind, chosen by each candidate's own
// seed, so the thinning is even and repeatable. Above the ceiling the plan is cut to
// exactly the ceiling in this order:
//   1. Mapped trees and tree rows (evidence "mapped-node/row") are kept first: they are
//      landmarks someone surveyed.
//   2. Bushes keep their share of what is left, at most ShrubShare of it, so shrub-heavy
//      ground keeps an understory without bushes taking the canopy's slots.
//   3. Trees take the rest. A kind that cannot fill its quota leaves its slots to the other.
// Within a kind the candidates with the lowest seed hashes are kept. That is the same
// order the slider's threshold uses, so the cut is spatially even and deterministic,
// while the count is exact rather than an expected value.
public static class VegetationBudget
{
    public const float ShrubShare = .25f;

    // TreesPlanned and ShrubsPlanned count what the slider left, mapped ones included.
    public readonly record struct Result(
        VegetationCandidate[] Kept,
        int Planned,
        int MappedKept,
        int TreesKept,
        int TreesPlanned,
        int ShrubsKept,
        int ShrubsPlanned,
        bool Capped);

    // The same hash the slider has always used, as a value in [0, 1).
    private static float Draw(VegetationCandidate candidate)
        => (math.hash(new uint2(candidate.Seed, 0x7E5u)) & 0xFFFF) / 65536f;

    public static Result Fit(VegetationPlan plan, float density, int ceiling)
    {
        var mapped = new List<int>();
        var trees = new List<int>();
        var shrubs = new List<int>();

        for (var i = 0; i < plan.Count; i++)
        {
            var candidate = plan[i];

            if (density < 1 && Draw(candidate) >= density)
                continue;

            var evidence = plan.Metadata.Count > candidate.MetadataIndex && candidate.MetadataIndex >= 0
                ? plan.Metadata[candidate.MetadataIndex].Evidence
                : "";

            if (evidence.StartsWith("mapped-node/row", StringComparison.Ordinal))
                mapped.Add(i);
            else if (candidate.Form == VegetationForm.Shrub)
                shrubs.Add(i);
            else
                trees.Add(i);
        }

        var total = mapped.Count + trees.Count + shrubs.Count;
        var treesPlanned = trees.Count + mapped.Count(index => plan[index].Form != VegetationForm.Shrub);
        var shrubsPlanned = shrubs.Count + mapped.Count(index => plan[index].Form == VegetationForm.Shrub);
        var capped = total > ceiling;

        if (capped)
        {
            // Lowest draw first; equal draws keep plan order.
            void Keep(List<int> indexes, int count)
            {
                if (count >= indexes.Count)
                    return;

                var chosen =
                    indexes
                        .OrderBy(index => Draw(plan[index]))
                        .ThenBy(index => index)
                        .Take(Math.Max(0, count))
                        .OrderBy(index => index)
                        .ToList();

                indexes.Clear();
                indexes.AddRange(chosen);
            }

            Keep(mapped, ceiling);
            var remaining = ceiling - mapped.Count;
            var proportional = (double)remaining * shrubs.Count / Math.Max(1, trees.Count + shrubs.Count);
            var shrubQuota = Math.Min(shrubs.Count, (int)Math.Min(remaining * ShrubShare, proportional));
            var treeQuota = Math.Min(trees.Count, remaining - shrubQuota);
            shrubQuota = Math.Min(shrubs.Count, remaining - treeQuota);
            Keep(trees, treeQuota);
            Keep(shrubs, shrubQuota);
        }

        var keep = new bool[plan.Count];

        foreach (var index in mapped.Concat(trees).Concat(shrubs))
            keep[index] = true;

        var kept = new VegetationCandidate[mapped.Count + trees.Count + shrubs.Count];
        var next = 0;

        for (var i = 0; i < plan.Count; i++)
            if (keep[i])
                kept[next++] = plan[i];

        return new Result(
            kept,
            plan.Count,
            mapped.Count,
            trees.Count + mapped.Count(index => plan[index].Form != VegetationForm.Shrub),
            treesPlanned,
            shrubs.Count + mapped.Count(index => plan[index].Form == VegetationForm.Shrub),
            shrubsPlanned,
            capped);
    }
}
