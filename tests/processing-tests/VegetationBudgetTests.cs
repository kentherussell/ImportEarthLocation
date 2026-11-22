using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

// The tree and bush budget (report 121 §16.3): mapped trees first, bushes at most a
// quarter of what is left, trees the rest, an exact count, and the slider unchanged.
static class VegetationBudgetTests
{
    public static void Run()
    {
        var metadata =
            new[] { new VegetationMetadata("mapped-node/row", ""), new VegetationMetadata("mapped-habitat", "") };

        var candidates = new List<VegetationCandidate>();
        uint seed = 1;

        void Add(int count, VegetationForm form, int metadataIndex)
        {
            for (var i = 0; i < count; i++)
                candidates.Add(new(new float3(seed, 0, 0), form, seed, math.hash(new uint2(seed++, 7)), metadataIndex));
        }

        Add(100, VegetationForm.Broadleaf, 0);
        Add(1000, VegetationForm.Conifer, 1);
        Add(1000, VegetationForm.Shrub, 1);
        var plan = new VegetationPlan(candidates.ToArray(), metadata);

        var capped = VegetationBudget.Fit(plan, 1, 1000);
        Check(capped.Capped && capped.Kept.Length == 1000, "The ceiling did not keep exactly its count");
        Check(capped.MappedKept == 100, "Mapped trees were thinned below the ceiling");
        Check(
            capped.ShrubsKept == 225 && capped.TreesKept == 775,
            $"Bushes did not get a quarter of the slots left after mapped trees ({capped.ShrubsKept} bushes)");

        Check(
            capped.Kept.SequenceEqual(VegetationBudget.Fit(plan, 1, 1000).Kept),
            "The vegetation budget is nondeterministic");

        Check(
            capped.Kept.Select(candidate => candidate.Seed).SequenceEqual(
                plan.Where(capped.Kept.Contains).Select(candidate => candidate.Seed)),
            "The vegetation budget changed plan order");

        var tight = VegetationBudget.Fit(plan, 1, 60);
        Check(
            tight.Kept.Length == 60 && tight.MappedKept == 60,
            "A ceiling below the mapped trees did not keep only mapped trees");

        var open = VegetationBudget.Fit(plan, 1, 5000);
        Check(!open.Capped && open.Kept.Length == plan.Count, "A plan under the ceiling was thinned");

        var half = VegetationBudget.Fit(plan, .5f, 5000);
        Check(
            half.Kept.Length is > 900 and < 1200 && half.Kept.All(plan.Contains),
            $"The density slider did not keep about half ({half.Kept.Length})");

        Console.WriteLine("PASS: vegetation budget keeps mapped trees, quarters bushes, exact ceiling, slider share.");
    }
}
