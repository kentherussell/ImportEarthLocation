using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

// Exercise the production matcher/rebaser, not a duplicate test-only algorithm.
internal static class NativeBatchTests
{
    private static readonly Dictionary<string, string> Empty = new(StringComparer.Ordinal);

    private static PreparedSegment Section(long a = 1, long b = 2, NetworkKind kind = NetworkKind.Road) =>
        new(10, a, b, new float3(0, 182.917435f, 0), new float3(40, 181.47049f, 0), kind,
            kind == NetworkKind.Road ? "primary" : "rail", 2, false, false, false, 0)
        { Control1 = new float3(10, 182.5f, 0), Control2 = new float3(30, 182, 0) };

    private static NativeBatchBindings.Edge<string> Edge(
        PreparedSegment segment,
        string entity = "edge",
        string firstText = "a",
        string secondText = "b",
        string prefab = "stock") =>
        new(
            entity,
            firstText,
            secondText,
            segment.Start - new float3(0, .83197f, 0),
            segment.End - new float3(0, .32787f, 0),
            segment.Kind,
            prefab);

    private static NativeBatchBindings.Batch<string> Match(
        PreparedSegment segment,
        IReadOnlyList<NativeBatchBindings.Edge<string>> edges,
        IReadOnlyDictionary<string, string>? committed = null) =>
        NativeBatchBindings.Match(
            new[] { new NativeBatchBindings.Expected<string>(segment, "stock") },
            edges,
            committed ?? Empty);

    private static void Refuses(Action action, string expected, string message)
    {
        try
        {
            action();
        }
        catch (InvalidOperationException exception)
        {
            Check(
                exception.Message.Contains(expected, StringComparison.Ordinal),
                message + ": wrong rejection: " + exception.Message);

            return;
        }

        throw new Exception(message + ": unsafe graph accepted");
    }

    public static void Run()
    {
        NativeOwnershipTests.Run();
        NativeOwnerLineageTests.Run();
        NativeUnitPartition();
        Check(
            NativeBatchBindings.Target("preview", "null", false, false) == "preview",
            "New native entity should survive as itself");

        Check(
            NativeBatchBindings.Target("preview", "original", true, false) == "original",
            "Ordinary native update must bind Original");

        Check(
            NativeBatchBindings.Target("preview", "original", true, true) == "preview",
            "Native replacement must not bind the deleted Original");

        var segment = Section();
        var edge = Edge(segment);
        var result = Match(segment, new[] { edge });
        Check(result.Edges.Count == 1 && result.Anchors.Count == 2, "Valid native height adjustment lost coverage");
        Check(
            result.Anchors["Road:1"].Entity == "a" && math.all(result.Anchors["Road:1"].Generated == edge.Start),
            "Source identity did not retain actual native anchor");

        Check(
            !math.all(edge.Start == segment.Start) && math.all(edge.Start.xz == segment.Start.xz),
            "Regression fixture must reproduce the old XYZ readback failure");

        var reversed =
            edge with { StartNode = edge.EndNode, EndNode = edge.StartNode, Start = edge.End, End = edge.Start };

        result = Match(segment, new[] { reversed });
        Check(
            !result.Edges[0].Forward && result.Anchors["Road:1"].Entity == "a",
            "Reversed native edge changes source direction");

        Check(
            Match(
                segment,
                new[] {
                    edge,
                    edge with {
                        Entity = "parallel",
                        StartNode = "p",
                        EndNode = "q",
                        Start = edge.Start + new float3(0, 0, .1f),
                        End = edge.End + new float3(0, 0, .1f)
                        }
                    }).Edges.Count
                == 1,
            "Nearby parallel road confused exact source pair");

        Check(
            Match(
                segment,
                new[] {
                    edge,
                    edge with { Entity = "rail", Kind = NetworkKind.Rail },
                    edge with { Entity = "asset", Prefab = "wrong-stock" }
                    }).Edges.Count
                == 1,
            "Rail or unselected asset confused road binding");

        Refuses(
            () => Match(segment, Array.Empty<NativeBatchBindings.Edge<string>>()),
            "0 qualified",
            "Missing generated edge");

        Refuses(
            () => Match(segment, new[] { edge with { Prefab = "wrong-stock" } }),
            "0 qualified",
            "Wrong selected native asset");

        Refuses(
            () => Match(segment, new[] { edge with { Kind = NetworkKind.Rail } }),
            "0 qualified",
            "Rail crossing is not a road junction");

        Refuses(
            () => Match(segment, new[] { edge with { Start = edge.Start + new float3(.001f, 0, 0) } }),
            "0 qualified",
            "Horizontal drift must not snap to nearest road");

        Refuses(
            () => Match(
                segment,
                new[] { edge, edge with { Entity = "ambiguous", StartNode = "other-a", EndNode = "other-b" } }),
            "2 qualified",
            "Ambiguous coincident asset pair");

        Refuses(
            () => Match(segment, new[] { edge with { Start = new float3(float.NaN, 0, 0) } }),
            "non-finite",
            "Non-finite generated endpoint");

        Refuses(
            () => Match(segment, new[] { edge with { End = new float3(40, float.PositiveInfinity, 0) } }),
            "non-finite",
            "Non-finite native height");

        Refuses(
            () => Match(segment, new[] { edge }, new Dictionary<string, string> { { "Road:1", "wrong-node" } }),
            "0 qualified",
            "Known source identity must not migrate to another entity");

        Refuses(
            () => Match(segment, new[] { edge }, new Dictionary<string, string> { { "Road:99", "a" } }),
            "merged unrelated",
            "A generated node cannot steal a different committed identity");

        Check(
            Match(
                segment,
                new[] { edge },
                new Dictionary<string, string> { { "Road:99", "old" }, { "Road:100", "old" } }).Edges.Count
                == 1,
            "An unrelated committed identity conflict poisoned a disjoint safe batch");

        Refuses(
            () => Match(
                segment,
                new[] { edge },
                new Dictionary<string, string> { { "Road:1", "a" }, { "Road:100", "a" } }),
            "conflicting source", "A required port used an ambiguously owned committed identity");

        var contracted = segment with { SourceNodeAliases = new[] { new SourceNodeAlias(100, 1, true) } };
        Check(
            Match(
                contracted,
                new[] { edge },
                new Dictionary<string, string> { { "Road:1", "a" }, { "Road:100", "a" } }).Edges.Count
                == 1,
            "An explicitly declared source contraction did not preserve verified many-source-to-one lineage");

        Refuses(
            () => Match(
                contracted,
                new[] { edge },
                new Dictionary<string, string> { { "Road:1", "a" }, { "Road:100", "other" } }),
            "conflicting source", "A contraction authorized two different native entities for one canonical port");

        Refuses(
            () => NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(segment, "stock")
                    },
                new[] { edge },
                Empty),
            "same native edge",
            "Duplicate edge coverage");

        var next =
            segment with
            {
                WayId = 11,
                StartNodeId = 2,
                EndNodeId = 3,
                Start = segment.End,
                End = new float3(80, 181, 0)
            };

        var nextEdge = Edge(next, "next", "b", "c") with { Start = edge.End };
        var connected =
            NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(next, "stock")
                    },
                new[] { edge, nextEdge },
                Empty);

        Check(
            connected.Anchors.Count == 3 && connected.Anchors["Road:2"].Entity == "b",
            "Shared junction split within one batch");

        Refuses(
            () => NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(next, "stock")
                    },
                new[] { edge, nextEdge with { StartNode = "duplicate-b" } },
                Empty),
            "0 qualified",
            "Shared source junction generated as two nodes");

        Refuses(
            () => NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(next, "stock")
                    },
                new[] { edge, nextEdge with { Start = nextEdge.Start + new float3(0, .1f, 0) } },
                Empty),
            "conflicting anchors",
            "One native entity at conflicting positions");

        var unrelated = next with { StartNodeId = 99 };
        Refuses(
            () => NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(unrelated, "stock")
                    },
                new[] { edge, nextEdge },
                Empty),
            "merged unrelated",
            "Distinct source nodes cannot become a false connection");

        var committed =
            connected.Anchors.ToDictionary(entry => entry.Key, entry => entry.Value.Entity, StringComparer.Ordinal);

        Check(
            Match(next, new[] { nextEdge }, committed).Anchors["Road:2"].Entity == "b",
            "Cross-batch junction lost explicit identity");

        var replacement =
            NativeBatchBindings.RemapCommitted(committed, new Dictionary<string, string> { { "b", "replacement-b" } });

        Check(
            Match(next, new[] { nextEdge with { StartNode = "replacement-b" } }, replacement).Anchors["Road:2"].Entity
                == "replacement-b"
                && committed["Road:2"] == "b",
            "Native replacement source remap mutated old bindings or lost lineage");

        var portal =
            next with
            {
                Bridge = true,
                Layer = 1,
                StartElevation = 0,
                EndElevation = 8,
                RetainingWallSides = NetworkSides.Left,
                StartCurveOffset = new float3(2, 0, 0)
            };

        Check(
            Match(portal, new[] { nextEdge }, committed).Edges.Count == 1,
            "Shared ground-to-bridge portal blocked by structure identity");

        Check(
            Match(portal with { Bridge = false, Tunnel = true, Layer = -1 }, new[] { nextEdge }, committed).Edges.Count
                == 1,
            "Shared ground-to-tunnel portal blocked");

        var rail = next with { Kind = NetworkKind.Rail };
        Check(
            Match(
                rail,
                new[] { nextEdge with { Kind = NetworkKind.Rail, StartNode = "rail-b", EndNode = "rail-c" } },
                committed).Anchors["Rail:2"].Entity
                == "rail-b",
            "Same road/rail numeric source ID must not join modes");

        var midpoint = new float3(20, 182, 0);
        var splitA = edge with { Entity = "split-a", EndNode = "subdivision", End = midpoint };
        var splitB = edge with { Entity = "split-b", StartNode = "subdivision", Start = midpoint };
        var split = Match(segment, new[] { splitB, splitA });
        Check(
            split.SectionCount == 1 && split.Edges.Count == 2 && split.Nodes.Count == 3 && split.Anchors.Count == 2,
            "Native subdivision not verified as one complete submitted span");

        Refuses(() => Match(segment, new[] { splitA }), "0 qualified", "Incomplete subdivided span");
        Refuses(
            () => Match(segment, new[] { splitA, splitB with { StartNode = "disconnected" } }),
            "0 qualified",
            "Spatially touching split edges with different node entities are not connected");

        Refuses(
            () => Match(segment, new[] { edge, splitA, splitB }),
            "2 qualified",
            "Direct edge and split path both present are ambiguous duplicate coverage");

        Refuses(
            () => Match(
                segment,
                new[] {
                    splitA with { End = midpoint + new float3(0, 0, 1) },
                    splitB with { Start = midpoint + new float3(0, 0, 1) }
                    }),
            "0 qualified",
            "Alternate route is not an anonymous native subdivision of submitted course");

        var bent = segment with { Control1 = new float3(10, 182.5f, 30), Control2 = new float3(30, 182, 30) };
        var curvedMidpoint = new float3(20, 182, 22.5f);
        Check(
            Match(
                bent,
                new[] { splitA with { End = curvedMidpoint }, splitB with { Start = curvedMidpoint } }).Edges.Count
                == 2,
            "Curved-course subdivision mistaken for a straight-line detour");

        Refuses(
            () => Match(
                segment,
                new[] { splitA, splitB },
                new Dictionary<string, string> { { "Road:999", "subdivision" } }),
            "0 qualified",
            "Cannot route a submitted span through another committed source junction");

        var longSource =
            segment with
            {
                Start = new float3(0, 182, 0),
                End = new float3(40960, 182, 0),
                Control1 = null,
                Control2 = null
            };

        var longChain =
            Enumerable
                .Range(0, 2048)
                .Select(
                    i => new NativeBatchBindings.Edge<string>(
                        "long-edge-" + i,
                        "long-node-" + i,
                        "long-node-" + (i + 1),
            new float3(i * 20, 181, 0), new float3((i + 1) * 20, 181, 0), NetworkKind.Road, "stock")).ToArray();

        Check(
            Match(longSource, longChain).Edges.Count == 2048,
            "Long legitimate native chain did not complete without recursive stack growth");

        SimulateBatches(53060);
        SimulateBatches(17999);
        LaterBatchRegressions();
        OverlapCoalescenceRegressions();
        ManifestRoundTrip();
        Console.WriteLine(
            "PASS: production native batch bindings recover height-only drift, preserve portals/mode/source identity and control handles, reject missing/ambiguous/parallel/aliased/non-finite graphs; 71,059 synthetic sections complete across 556 batches (not a native-game import).");
    }

    private static void NativeUnitPartition()
    {
        var log = new ImportLog();

        PreparedSegment Road(long way, long a, long b, float3 start, float3 end) =>
            new(way, a, b, start, end, NetworkKind.Road, "motorway", 2, true, false, false, 0)
            { NativeWidth = 8, NativeAssetKey = "two" };

        var assets =
            new[] {
                new NativeAssetContract(
                    "two",
                    new[] { new NativeCompositionEnvelope(NativeStructureMode.Ground, 0, 8, 0, 0, 0, 0, 4, true) },
                    Array.Empty<float>())
                };

        var source = new[]{Road(10,1,2,new(-200,20,0),new(0,20,0)),Road(11,2,3,new(0,20,0),new(200,20,0)),
            Road(12,2,4,new(0,20,0),new(0,20,200))};

        var partition = NativeDesignUnits.Prepare(source, assets, log, default);
        var junction = partition.Where(segment => segment.StartNodeId == 2 || segment.EndNodeId == 2).ToArray();
        Check(
            junction.Length == 3 && junction.Select(segment => segment.NativeUnitId).Distinct().Count() == 1,
            "A source junction's native prefixes split across units");

        Check(partition.Count == 6 && partition.Select(segment => segment.NativeUnitId).Distinct().Count() == 4
            && junction.All(segment => math.distance(segment.CourseStart, segment.CourseEnd) > 4),
            "Single-span handoffs lost distinct terminal/junction ownership or retained redundant corridor cuts");

        foreach (var group in partition.GroupBy(segment => segment.WayId))
        {
            var pieces = group.ToArray();
            var original = source.Single(segment => segment.WayId == group.Key);
            Check(
                math.all(pieces[0].CourseStart == original.CourseStart)
                    && math.all(pieces[^1].CourseEnd == original.CourseEnd),
                "Unit cut moved external float ports");

            for (var i = 1; i < pieces.Length; i++)
                Check(
                    pieces[i - 1].EndNodeId == pieces[i].StartNodeId
                        && math.all(pieces[i - 1].End == pieces[i].Start)
                        && math.all(pieces[i - 1].CourseEnd == pieces[i].CourseStart),
                    "Unit boundary independently rounded its shared anchor");

            Check(
                Math.Abs(pieces.Sum(segment => math.distance(segment.CourseStart, segment.CourseEnd)) - 200) < .001,
                "Exact straight subdivision changed coverage");
        }

        var plan = new NativeSubmissionPlan(partition);
        var seen = new HashSet<int>();
        var progress = 0;

        while (progress < plan.Sections.Count)
        {
            var batch = plan.Batch(progress, 2);
            Check(batch.Count > 0, "Atomic scheduler stalled");

            foreach (var unit in batch.GroupBy(segment => segment.NativeUnitId))
            {
                Check(seen.Add(unit.Key), "One native unit appeared in multiple batches");
                Check(
                    unit.Count() == partition.Count(segment => segment.NativeUnitId == unit.Key),
                    "Batch truncated an indivisible junction unit");
            }

            progress += batch.Count;
        }

        var shortBridge =
            Road(13, 2, 5, new(0, 20, 0), new(10, 20, 0)) with { Bridge = true, NativeAssetKey = "bridge" };

        var joined =
            NativeDesignUnits.Prepare(
                source.Concat(new[] { shortBridge, Road(14, 5, 6, new(10, 20, 0), new(210, 20, 0)) }).ToArray(),
                assets,
                log,
                default);

        Check(
            joined
                .Where(
                    segment => segment.StartNodeId == 2
                        || segment.EndNodeId == 2
                        || segment.StartNodeId == 5
                        || segment.EndNodeId == 5)
                .Select(segment => segment.NativeUnitId)
                .Distinct()
                .Count()
                == 1,
            "Overlapping structure-transition cores were not unioned");

        var dense =
            new NativeSubmissionPlan(
                Enumerable.Range(0, 65).Select(i => source[0] with { WayId = i, NativeUnitId = 7 }).ToArray());

        Check(
            dense.Batch(0, 32).Count == 65 && dense.GatherPendingJunction(0, 65, 100, new[] { "Road:1" }) == 0,
            "Oversized core was split or legacy recovery mutated it");

        Refuses(() => dense.Batch(1, 32), "inside", "Scheduling inside a native unit");
        var longRoad =
            Enumerable
                .Range(0, 200)
                .Select(i => Road(100 + i, 1000 + i, 1001 + i, new(i * 10, 20, 0), new((i + 1) * 10, 20, 0)))
                .ToArray();

        var corridor = NativeDesignUnits.Prepare(longRoad, assets, log, default);
        Check(
            corridor.GroupBy(segment => segment.NativeUnitId).Max(group => group.Count()) <= 33
                && corridor
                    .Select(segment => segment.WayId)
                    .ToHashSet()
                    .SetEquals(longRoad.Select(segment => segment.WayId))
                && Math.Abs(corridor.Sum(segment => math.distance(segment.CourseStart, segment.CourseEnd)) - 2000)
                    < .01,
            "A long corridor became an unnecessarily city-sized indivisible native unit");

        var unsafeCurve =
            NativeDesignUnits.Prepare(
                new[] { source[0] with { Control1 = new(float.NaN, 0, 0) } },
                assets,
                log,
                default);

        Check(
            unsafeCurve.Count == 1 && !math.all(math.isfinite(unsafeCurve[0].CourseControl1)),
            "Nonfinite unit geometry was recursively repaired or silently certified");

        Console.WriteLine(
            "PASS: exact native core/corridor cuts, shared float ports, short-core union, complete unit packing and oversized indivisible units; engine acceptance remains unexercised.");
    }

    private static void LaterBatchRegressions()
    {
        var segment =
            Section() with
            {
                Start = new float3(15099.667f, 229.35846f, -11503.515f),
                End = new float3(15083.043f, 229.27856f, -11503.26f),
                Control1 = null,
                Control2 = null,
                NativeWidth = 32
            };

        var center = new float3(15094.125f, 229.31899f, -11503.43f);
        var old =
            new NativeBatchBindings.Anchor<string>(
                "committed",
                segment.Start,
                segment.Start - new float3(0, .03947f, 0));

        // A revised ramp attaches to a node the game moved slightly, within the usual budget.
        var limited = old with { CenterLimit = 8 };
        var revision = NativeBatchBindings.LocalRevisionAnchor(limited, segment.Start + new float3(3, .5f, 0));
        Check(
            math.all(revision.Generated == limited.Generated) && revision.Planned.x == segment.Start.x + 3,
            "Local revision within the node budget was refused or moved the live node");

        Refuses(
            () => NativeBatchBindings.LocalRevisionAnchor(limited, segment.Start + new float3(9, 0, 0)),
            "Local revision",
            "Local revision beyond the node budget was accepted");

        var bound = new Dictionary<string, string> { { "Road:1", "committed" } };
        var edge =
            new NativeBatchBindings.Edge<string>(
                "new-edge",
                "committed",
                "new-end",
                center,
                segment.End,
                segment.Kind,
                "stock")
            { CurveStart = segment.CourseStart, CurveEnd = segment.CourseEnd };

        var valid = Match(segment, new[] { edge }, bound);
        Check(
            math.all(valid.Anchors["Road:1"].Generated == center),
            "Native center regeneration lost exact submitted curve-port identity");

        Refuses(
            () => Match(segment, new[] { edge with { CurveStart = null, CurveEnd = null } }, bound),
            "0 qualified",
            "Center displacement without native curve evidence");

        Refuses(
            () => Match(segment, new[] { edge with { StartNode = "wrong-owner" } }, bound),
            "0 qualified",
            "Curve evidence must not bypass committed ownership");

        Refuses(
            () => Match(segment, new[] { edge with { Start = center + new float3(50, 0, 0) } }, bound),
            "0 qualified",
            "Center moved beyond native footprint");

        Refuses(
            () => Match(segment, new[] { edge with { CurveStart = segment.Start + new float3(.01f, 0, 0) } }, bound),
            "0 qualified",
            "Nearby curve port is not exact submitted port");

        Refuses(
            () => Match(segment, new[] { edge with { CurveEnd = null } }, bound),
            "incomplete curve",
            "Partial native curve evidence");

        var narrow = segment with { NativeWidth = 12 };
        var widerCenter = center - new float3(5, 0, 0);
        var wideJunctionEdge = edge with { Start = widerCenter };
        Refuses(
            () => Match(narrow, new[] { wideJunctionEdge }, bound),
            "0 qualified",
            "Narrow road may not authorize unproven large center displacement");

        var wideJunction =
            NativeBatchBindings.Match(
                new[] { new NativeBatchBindings.Expected<string>(narrow, "stock") { StartCenterBudget = 18 } },
                new[] { wideJunctionEdge },
                bound);

        var sharedAnchor = wideJunction.Anchors["Road:1"] with { CenterLimit = 18 };
        Check(
            math.all(
                NativeBatchBindings.RebaseBound(narrow, sharedAnchor, null).CourseStart.xz == narrow.CourseStart.xz),
            "Widest incident source footprint must consistently authorize continuation onto a narrower arm");

        var anchor = valid.Anchors["Road:1"];
        var continuation = NativeBatchBindings.RebaseBound(segment, anchor, null);
        Check(
            math.all(continuation.Start == center)
                && math.all(continuation.CourseStart.xz == segment.CourseStart.xz)
                && math.all(continuation.CourseControl1.xz == segment.CourseControl1.xz),
            "Continuation must distinguish native center from fixed source curve ports/handles");

        Check(
            math.all(continuation.CourseControl2.xz == segment.CourseControl2.xz)
                && math.all(continuation.CourseEnd.xz == segment.CourseEnd.xz),
            "Rebasing one junction moved the other port");

        Check(
            Match(continuation, new[] { edge }, bound).Edges.Count == 1,
            "Actual rebased course manifest cannot match generated curve ports");

        Refuses(
            () => NativeBatchBindings.RebaseBound(
                segment,
                anchor with { Planned = anchor.Planned + new float3(1, 0, 0) },
                null),
            "provenance",
            "Cannot authorize rebase with unrelated source point");

        var next =
            segment with
            {
                WayId = 11,
                StartNodeId = 2,
                EndNodeId = 3,
                Start = segment.End,
                End = new float3(14863.046f, 229.17683f, -11500.373f),
                Control1 = null,
                Control2 = null
            };

        var bypass =
            new NativeBatchBindings.Edge<string>(
                "bypass",
                "committed",
                "next-end",
                center,
                next.End,
                segment.Kind,
                "stock")
            { CurveStart = next.CourseStart, CurveEnd = next.CourseEnd };

        Refuses(
            () => NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(next, "stock")
                    },
                new[] { edge, bypass },
                bound),
            "0 qualified",
            "Captured Palm Springs bypass still must be rejected before Apply");

        var joined = bypass with { StartNode = "new-end", Start = segment.End };
        Check(
            NativeBatchBindings.Match(
                new[] {
                    new NativeBatchBindings.Expected<string>(segment, "stock"),
                    new NativeBatchBindings.Expected<string>(next, "stock")
                    },
                new[] { edge, joined },
                bound).Anchors.Count
                == 3,
            "Corrected later-batch incidence failed");

        Check(
            NativePreviewRecovery.NextBatchSize(128, 0, false, true) == 32
                && NativePreviewRecovery.NextBatchSize(32, 1, false, true) == 1
                && NativePreviewRecovery.NextBatchSize(1, 2, false, true) == 0,
            "Recovery must change context; repeating an identical single-section preview cannot repair it");

        Check(
            NativePreviewRecovery.NextBatchSize(128, 0, true, true) == 0
                && NativePreviewRecovery.NextBatchSize(128, 0, false, false) == 0
                && NativePreviewRecovery.NextBatchSize(1, 3, false, true) == 0,
            "Applied, unclassified or exhausted failures may not be replayed");

        Console.WriteLine(
            "PASS: later-batch 5.54 m center/curve-port separation, bound continuation, correct selected entity/asset, captured bypass rejection and bounded pre-commit-only retries.");
    }

    private static void OverlapCoalescenceRegressions()
    {
        // 2026-10-09 log-derived failure shapes. Correct source flags must
        // prevent these previews; the matcher must NOT start accepting them
        // just to make old captured batches or an import report success.
        var section = Section() with { NativeWidth = 32 };
        var owner = new Dictionary<string, string> { { "Road:1", "owned-start" } };
        var unchanged =
            Edge(
                section,
                "continuation",
                "owned-start",
                "new-end") with
            { CurveStart = section.CourseStart, CurveEnd = section.CourseEnd };

        Check(
            Match(section, new[] { unchanged }, owner).SectionCount == 1,
            "Protected continuation onto an unchanged committed owner rejected");

        foreach (var displacement in new[] { new float3(-7.59277f, 0, -2.5205f), new float3(-15.81836f, 0, 2.4023f) })
        {
            var recentered =
                unchanged with
                {
                    Start = unchanged.Start + displacement,
                    CurveStart = section.CourseStart + displacement
                };

            Refuses(
                () => Match(section, new[] { recentered }, owner),
                "0 qualified",
                "Moved owner AND moved curve port may not be accepted as valid continuation");
        }

        var shortSection =
            section with
            {
                Start = new float3(14622.9277f, 182, -6361.80225f),
                End = new float3(14541.6309f, 182, -6362.53027f),
                Control1 = null,
                Control2 = null
            };

        var mergedEnd = new float3(14420.4307f, 181, -6363.607f);
        var merged =
            new NativeBatchBindings.Edge<string>(
                "merged-202m",
                "owned-start",
                "different-owned-junction",
                shortSection.Start,
                mergedEnd,
                NetworkKind.Road,
                "stock")
            { CurveStart = shortSection.CourseStart, CurveEnd = mergedEnd };

        var owners =
            new Dictionary<string, string> { { "Road:1", "owned-start" }, { "Road:99", "different-owned-junction" } };

        Refuses(
            () => Match(shortSection, new[] { merged }, owners),
            "0 qualified",
            "Native 81m-to-202m extension deleting an intermediate source junction must remain unsafe");

        Console.WriteLine(
            "PASS: unchanged explicit continuation accepted; log-derived owner/port movement and overextended known-junction bypass rejected. This validates matcher safety, not native merge prevention or whole-import completion.");
    }

    private static void SimulateBatches(int sectionCount)
    {
        var committed = new Dictionary<string, string>(StringComparer.Ordinal);
        var total = 0;
        var batches = 0;

        float3 Position(int id) => new(-24000 + (id % 800) * 60, 200, -24000 + (id / 800) * 60);

        string Node(NetworkKind kind, int id) => $"{kind}-entity-{id}";

        float3 Generated(float3 point, int id) => point + new float3(0, -.832f + (id % 17) * .05f, 0);

        for (var first = 0; first < sectionCount; first += 128)
        {
            var expected = new List<NativeBatchBindings.Expected<string>>();
            var actual = new List<NativeBatchBindings.Edge<string>>();

            for (var i = first; i < Math.Min(first + 128, sectionCount); i++)
            {
                var kind = i % 2000 < 1000 ? NetworkKind.Road : NetworkKind.Rail;
                var source =
                    Section(
                        i,
                        i + 1,
                        kind) with
                    { WayId = i + 1, Start = Position(i), End = Position(i + 1), Control1 = null, Control2 = null };

                var vectorA = Generated(source.Start, i);
                var vectorB = Generated(source.End, i + 1);
                expected.Add(new(source, "stock"));
                actual.Add(new("edge-" + i, Node(kind, i), Node(kind, i + 1), vectorA, vectorB, kind, "stock"));
            }

            // Native archetype enumeration is not submission order.
            actual.Reverse();
            var result = NativeBatchBindings.Match(expected, actual, committed);
            Check(result.Edges.Count == expected.Count, "Many-batch import lost section coverage");

            foreach (var entry in result.Anchors)
            {
                if (committed.TryGetValue(entry.Key, out var prior))
                    Check(prior == entry.Value.Entity, "Batch boundary spawned duplicate node");

                committed[entry.Key] = entry.Value.Entity;
            }

            total += result.Edges.Count;
            batches++;
        }

        Check(
            total == sectionCount && batches == (sectionCount + 127) / 128,
            "Whole many-batch handoff model did not complete");
    }

    public static void Replay(string folder)
    {
        var manifests = CapturedRunFiles.Names(folder, "native-batch-", "-expected.ndjson");

        if (manifests.Length > 0)
        {
            ReplayManifests(folder, manifests);
            return;
        }

        var source =
            File.ReadLines(Path.Combine(folder, "native-plan-network.ndjson")).Take(128).Select(ParseSection).ToArray();

        Check(source.Length == 128, "Captured first batch is incomplete");
        var generated = new List<NativeBatchBindings.Edge<string>>();

        foreach (var line in File.ReadLines(Path.Combine(folder, "last-native-temp-batch.ndjson")))
        {
            var row = JObject.Parse(line);

            if (row["edge"] is not JObject otherRow)
                continue;

            Check(
                (string?)row["original"] == "0:0",
                "Replay is first-create-batch only; cannot infer unrecorded node originals for later batches");

            var kind =
                ((string?)row["prefabName"] ?? "").Contains("Track", StringComparison.OrdinalIgnoreCase)
                    ? NetworkKind.Rail
                    : NetworkKind.Road;

            generated.Add(
                new(
                    (string)row["entity"]!,
                    (string)otherRow["start"]!,
                    (string)otherRow["end"]!,
                    VisualMetricsReplay.ReadPoint(otherRow["startPosition"]!),
                    VisualMetricsReplay.ReadPoint(otherRow["endPosition"]!),
                    kind,
                    (string)row["prefab"]!));
        }

        // The old archive records generated assets, not the resolver's expected
        // prefab IDs. Qualify replay geometry first; production additionally
        // requires the actual resolver result (wrong-asset fixtures above).
        var expected = new List<NativeBatchBindings.Expected<string>>();

        foreach (var segment in source)
        {
            var candidates =
                generated
                    .Where(
                        edge => edge.Kind == segment.Kind
                            && (math.all(edge.Start.xz == segment.Start.xz)
                                || math.all(edge.End.xz == segment.Start.xz)))
                    .Select(edge => edge.Prefab)
                    .Distinct()
                    .ToArray();

            var qualified = new List<string>();

            foreach (var prefab in candidates)
            {
                try
                {
                    NativeBatchBindings.Match(
                        new[] { new NativeBatchBindings.Expected<string>(segment, prefab) },
                        generated,
                        Empty);

                    qualified.Add(prefab);
                }
                catch (InvalidOperationException) {/* No unique complete chain with this recorded asset. */}
            }

            Check(
                qualified.Count == 1,
                $"Replay lacks uniquely recorded chain asset: way={segment.WayId}; candidates={candidates.Length}; qualified={qualified.Count}");

            expected.Add(new(segment, qualified[0]));
        }

        var bindings = NativeBatchBindings.Match(expected, generated, Empty);
        var changed = bindings.Anchors.Values.Count(anchor => !math.all(anchor.Planned == anchor.Generated));
        Check(
            bindings.SectionCount == 128 && bindings.Edges.Count >= 128 && changed > 0,
            "Real regression did not recover height-adjusted first-batch graph");

        Console.WriteLine(
            $"PASS: captured {Path.GetFileName(folder)} first batch: {bindings.SectionCount}/128 source sections, {bindings.Edges.Count} native edges; {bindings.Anchors.Count} source anchors, {changed} differ from prepared XYZ, {bindings.Nodes.Count - bindings.Anchors.Count} native subdivision nodes. Native simulation and resolver asset selection are not replayed.");
    }

    private static PreparedSegment ParseSection(string line)
    {
        var row = JObject.Parse(line);
        return new(
            (long)row["WayId"]!,
            (long)row["StartNodeId"]!,
            (long)row["EndNodeId"]!,
            VisualMetricsReplay.ReadPoint(row["Start"]!),
            VisualMetricsReplay.ReadPoint(row["End"]!),
                    Enum.Parse<NetworkKind>((string)row["Kind"]!),
                        (string)row["Classification"]!,
                        (int)row["Lanes"]!,
                        (bool)row["OneWay"]!,
                        (bool)row["Bridge"]!,
                        (bool)row["Tunnel"]!,
                        (int)row["Layer"]!)
        {
            Control1 = row["Control1"] is JObject c1 ? VisualMetricsReplay.ReadPoint(c1) : null,
            Control2 = row["Control2"] is JObject c2 ? VisualMetricsReplay.ReadPoint(c2) : null,
            StartCurveOffset =
                row["StartCurveOffset"] is JObject firstRecord
                    ? VisualMetricsReplay.ReadPoint(firstRecord)
                    : float3.zero,
            EndCurveOffset =
                row["EndCurveOffset"] is JObject secondRecord
                    ? VisualMetricsReplay.ReadPoint(secondRecord)
                    : float3.zero,
            NativeWidth = (float?)row["NativeWidth"] ?? 0,
            ConsolidatedAvenue = (bool?)row["ConsolidatedAvenue"] ?? false,
            Name = (string?)row["Name"] ?? "",
            Reference = (string?)row["Reference"] ?? "",
            Roundabout = (bool?)row["Roundabout"] ?? false,
            StartMiniRoundabout = (bool?)row["StartMiniRoundabout"] ?? false,
            EndMiniRoundabout = (bool?)row["EndMiniRoundabout"] ?? false,
            SourceWayIds = row["SourceWayIds"]?.Values<long>().ToArray() ?? Array.Empty<long>(),
            SourceGeometry =
                row["SourceGeometry"]?.Select(VisualMetricsReplay.ReadPoint).ToArray() ?? Array.Empty<float3>(),
            OriginalGeometry =
                row["OriginalGeometry"]?.Select(VisualMetricsReplay.ReadPoint).ToArray() ?? Array.Empty<float3>()
        };
    }

    public static void ReplayJunctionPlan(string folder)
    {
        var network = Path.Combine(folder, "native-plan-network.ndjson");
        var meta = JObject.Parse(File.ReadAllText(network + ".meta.json"));

        if ((bool?)meta["Truncated"] ?? true)
            throw new InvalidOperationException("Incomplete junction-plan artifact: " + network);

        var source = File.ReadLines(network).Select(line =>
        {
            if ((bool?)JObject.Parse(line)["SourcePointsTruncated"] ?? false)
                throw new InvalidOperationException("Incomplete junction source geometry: " + network);

            return ParseSection(line);
        }).ToArray();

        Check(source.Length == (int)meta["Records"]!, "Junction-plan metadata count differs from recorded sections");
        var palm =
            source.FirstOrDefault(
                segment => segment.WayId == 1220190819
                    && segment.StartNodeId == -7999999999999999208
                    && segment.EndNodeId == -7999999999999999209);

        var oman =
            source.FirstOrDefault(
                segment => segment.WayId == 183177670
                    && segment.StartNodeId == -7999999999999998872
                    && segment.EndNodeId == -7999999999999999839);

        var link =
            palm
                ?? oman ?? throw new Exception(
                    "Archive does not contain either diagnosed later-batch median junction.");

        var arms =
            source
                .Where(
                    segment => segment.ConsolidatedAvenue
                        && (segment.StartNodeId == link.StartNodeId
                            || segment.EndNodeId == link.StartNodeId
                            || segment.StartNodeId == link.EndNodeId
                            || segment.EndNodeId == link.EndNodeId))
                .ToArray();

        var log = new ImportLog();
        var repaired = AvenueJunctionRepair.Prepare(source, log, default);
        Check(
            !repaired.Any(
                segment => segment.WayId == link.WayId
                    && segment.StartNodeId == link.StartNodeId
                    && segment.EndNodeId == link.EndNodeId),
            "Real archived connector was not recovered");

        var canonical = Math.Min(link.StartNodeId, link.EndNodeId);
        Check(
            repaired.Any(
                segment => segment.ConsolidatedAvenue
                    && (segment.StartNodeId == canonical || segment.EndNodeId == canonical)),
            "Repaired archived junction lost its avenue arms");

        foreach (var arm in arms.Where(
            segment => segment.StartNodeId != link.StartNodeId && segment.StartNodeId != link.EndNodeId
                || segment.EndNodeId != link.StartNodeId && segment.EndNodeId != link.EndNodeId))
            Check(
                repaired.Any(
                    segment => segment.WayId == arm.WayId
                        && (segment.StartNodeId == canonical || segment.EndNodeId == canonical)),
                "An archived approach lost its canonical junction");

        Check(
            !repaired.Any(segment => segment.StartNodeId == segment.EndNodeId),
            "Archived topology repair created a self-loop");

        Check(
            AvenueJunctionRepair.Prepare(repaired, new ImportLog(), default).SequenceEqual(repaired),
            "Real archived repair is not idempotent");

        Console.WriteLine(
            $"PASS: {Path.GetFileName(folder)} real complete archived plan ({source.Length} sections) recovers diagnosed way {link.WayId}; repaired sections={repaired.Count}. Offline topology evidence only; archived native preview is not regenerated and terrain must be reaudited in production.");

        foreach (var line in log.Lines.Where(
            text => text.Contains("connectorWay=" + link.WayId + ";", StringComparison.Ordinal)))
            Console.WriteLine(line);
    }

    private static void ReplayManifests(string folder, IReadOnlyList<string> manifests)
    {
        var accepted = 0;
        var rejected = 0;
        var pending = 0;

        foreach (var name in manifests)
        {
            var stem = Path.GetFileName(name).Replace("-expected.ndjson", "");
            var manifest = CapturedRunFiles.Resolve(folder, Path.GetFileName(name));
            var graphPath = CapturedRunFiles.Optional(folder, stem + "-graph.ndjson");
            var ownersPath = CapturedRunFiles.Optional(folder, stem + "-owners.ndjson");

            if (graphPath == null || ownersPath == null)
            {
                pending++;
                continue;
            }

            var rejectionPath = CapturedRunFiles.Optional(folder, stem + "-matcher-rejection.ndjson");
            var logicalNames = new[] { Path.GetFileName(name), stem + "-graph.ndjson", stem + "-owners.ndjson" }
                .Concat(rejectionPath == null ? Array.Empty<string>() : new[] { stem + "-matcher-rejection.ndjson" });

            foreach (var logical in logicalNames)
            {
                var meta = CapturedRunFiles.Optional(folder, logical + ".meta.json");

                if (meta != null && ((bool?)JObject.Parse(File.ReadAllText(meta))["Truncated"] ?? false))
                    throw new InvalidOperationException("Incomplete batch artifact: " + logical);
            }

            var expected = File.ReadLines(manifest).Select(line =>
{
    var row = JObject.Parse(line);
    return new NativeBatchBindings.Expected<string>(ParseSection(row["section"]!.ToString()), (string)row["prefab"]!)
    {
        StartCenterBudget = (float?)row["StartCenterBudget"] ?? 0,
        EndCenterBudget = (float?)row["EndCenterBudget"] ?? 0
    };
}).ToArray();

            var graph = File.ReadLines(graphPath).Select(line =>
{
    var row = JObject.Parse(line);
    return new NativeBatchBindings.Edge<string>(
        (string)row["entity"]!,
        (string)row["startNode"]!,
        (string)row["endNode"]!,
        VisualMetricsReplay.ReadPoint(row["start"]!),
        VisualMetricsReplay.ReadPoint(row["end"]!),
        Enum.Parse<NetworkKind>((string)row["mode"]!),
        (string)row["prefab"]!)
    {
        CurveStart = row["curveStart"] is JObject firstRecord ? VisualMetricsReplay.ReadPoint(firstRecord) : null,
        CurveEnd = row["curveEnd"] is JObject secondRecord ? VisualMetricsReplay.ReadPoint(secondRecord) : null
    };
}).ToArray();

            var owners =
                File
                    .ReadLines(ownersPath)
                    .Select(JObject.Parse)
                    .ToDictionary(row => (string)row["source"]!, row => (string)row["entity"]!, StringComparer.Ordinal);

            var journal = CapturedRunFiles.Optional(folder, "checkpoints.ndjson");
            var wasRejected = rejectionPath != null
                || journal != null
                    && File
                        .ReadLines(journal)
                        .Select(JObject.Parse)
                        .Any(
                            row => (string?)row["phase"] == "native-batch/preview-rejected"
                                && (string?)row["details"]?["artifactStem"] == stem);

            try
            {
                var result = NativeBatchBindings.Match(expected, graph, owners);
                Check(!wasRejected, "Previously rejected captured preview now accepted without regeneration: " + stem);
                accepted++;
                Check(result.SectionCount == expected.Length, "Captured batch source coverage incomplete");
            }
            catch (NativePreviewException)
            {
                Check(wasRejected, "Captured accepted preview unexpectedly rejected: " + stem);
                rejected++;
            }
        }

        Check(accepted + rejected > 0, "No complete batch manifests to replay");
        Console.WriteLine(
            $"PASS: selected-asset/submitted-course/owner replay: accepted={accepted}; correctly rejected previews={rejected}; incomplete/pending manifests={pending}. Not a native-game import.");
    }

    private static void ManifestRoundTrip()
    {
        var root = Path.Combine(Path.GetTempPath(), "iel-native-manifest-" + Guid.NewGuid().ToString("N"));

        try
        {
            var archive = new RunArchive(root, new { fixture = true }, typeof(NativeBatchTests).Assembly.Location);
            archive.ExpectNativeSections(new[] { "Road:10:1:2" });
            archive.WriteCompressedRecords(
                "native-unit-final-fixture.ndjson.gz",
                new object[] { new { entity = "edge" } });

            archive.WriteCompressedRecords("native-unit-final-fixture-port-movements.ndjson.gz", Array.Empty<object>());
            archive.ObserveNativeSections("native-unit-final-fixture", new[] { ("Road:10:1:2", true) });
            Check(
                (bool)JObject.FromObject(archive.CaptureStatus())["completeNativeSectionCapture"]!,
                "Complete gzip native tables were incorrectly classified as missing chunk manifests");

            archive.RecordFailure("native-unit-final-fixture.ndjson.gz", new IOException("Failed later capture"));
            Check(
                !(bool)JObject.FromObject(archive.CaptureStatus())["completeNativeSectionCapture"]!,
                "Failed later capture retained stale complete coverage");

            archive.WriteCompressedRecords(
                "native-unit-final-fixture.ndjson.gz",
                new object[] { new { entity = "replacement" } });

            Check(
                (bool)JObject.FromObject(archive.CaptureStatus())["completeNativeSectionCapture"]!,
                "Successful recapture did not restore artifact coverage");

            for (var i = 0; i < 130; i++)
                archive.RecordFailure("other-failure-" + i, new IOException("Bounded ledger fixture"));

            archive.RecordFailure(
                "native-unit-final-fixture.ndjson.gz",
                new IOException("Failure after message ledger saturation"));

            Check(
                !(bool)JObject.FromObject(archive.CaptureStatus())["completeNativeSectionCapture"]!,
                "Saturated failure message ledger prevented coverage invalidation");

            archive.WriteCompressedRecords(
                "native-unit-final-fixture.ndjson.gz",
                new object[] { new { entity = "replacement-after-saturation" } });

            var records = Enumerable.Range(0, 600).Select(i => (object)new { index = i }).ToArray();
            var full = archive.WriteRecords("over-64.ndjson", records);
            Check(full.Records == 600 && !full.Truncated, "Diagnostic writer still imposes the old 64-record cap");
            var chunks = archive.WriteRecordChunks("chunked.ndjson", records);
            Check(
                chunks.Records == 600 && !chunks.Truncated && chunks.Chunks.Length == 3,
                "Chunked readback lost record coverage");

            Check(
                chunks.Chunks.All(result => result.Records <= 256)
                    && chunks.Chunks.Sum(result => result.Records) == 600,
                "Diagnostic chunk limits or cumulative counts are incorrect");

            var sequence =
                chunks.Chunks
                    .SelectMany(result => File.ReadLines(Path.Combine(archive.DirectoryPath, result.File)))
                    .Select(line => (int)JObject.Parse(line)["index"]!)
                    .ToArray();

            Check(
                sequence.SequenceEqual(Enumerable.Range(0, 600)),
                "Chunked diagnostic readback reordered or duplicated records");

            var compressedChunks =
                archive.WriteRecordChunks("compressed-chunked.ndjson", records, final: true, compress: true);

            var compressedSequence = new List<int>();

            foreach (var chunk in compressedChunks.Chunks)
            {
                Check(
                    chunk.Encoding == "gzip-ndjson" && chunk.File.EndsWith(".gz"),
                    "Compressed chunk manifest omitted its decoder contract");

                using var file = File.OpenRead(Path.Combine(archive.DirectoryPath, chunk.File));
                using var gzip =
                    new System.IO.Compression.GZipStream(file, System.IO.Compression.CompressionMode.Decompress);

                using var reader = new StreamReader(gzip);
                string? line;

                while ((line = reader.ReadLine()) != null)
                    compressedSequence.Add((int)JObject.Parse(line)["index"]!);
            }

            Check(
                !compressedChunks.Truncated && compressedSequence.SequenceEqual(Enumerable.Range(0, 600)),
                "Compressed chunk roundtrip lost ordered coverage");

            var segment = Section() with { NativeWidth = 32 };

            void Write(string stem, bool valid)
            {
                archive.WriteRecords(
                    stem + "-expected.ndjson",
                    new object[] {
                        new {
                            kind = "expected-course",
                            prefab = "stock",
                            StartCenterBudget = 18,
                            EndCenterBudget = 18,
                            section = DiagnosticArtifacts.NetworkRecords(new[] { segment }).First()
                            }
                        });

                archive.WriteRecords(
                    stem + "-owners.ndjson",
                    new object[] { new { source = "Road:1", entity = "committed" } });

                archive.WriteRecords(
                    stem + "-graph.ndjson",
                    valid
                        ? new object[]{
                            new{
                                entity="edge",
                                startNode="committed",
                                endNode="new-end",
                                start=DiagnosticArtifacts.Point(segment.Start+new float3(2,-.5f,0)),
                                end=DiagnosticArtifacts.Point(segment.End),
                    mode="Road",
                        prefab="stock",
                        curveStart=DiagnosticArtifacts.Point(segment.CourseStart),
                        curveEnd=DiagnosticArtifacts.Point(segment.CourseEnd)
                        }}
                        : Array.Empty<object>());

                if (!valid)
                    archive.WriteRecords(
                        stem + "-matcher-rejection.ndjson",
                        new object[] {
                            new { artifactStem = stem, applied = false, reason = "fixture-preview-rejected" }
                            });
            }

            Write("native-batch-000128-128-attempt-00", false);
            Write("native-batch-000128-032-attempt-01", true);
            var longStem = "native-batch-" + new string('x', 180);
            Write(longStem + "-attempt-00", false);
            Write(longStem + "-attempt-01", true);
            archive.Write(Array.Empty<string>(), new Dictionary<string, double>(), null);
            Check(CapturedRunFiles.Names(archive.DirectoryPath, "native-batch-", "-expected.ndjson").Length == 4
                && !File.Exists(Path.Combine(archive.DirectoryPath, longStem + "-attempt-00-expected.ndjson")),
                "Long logical native batch names were not shortened or discoverable through the artifact catalog");

            Replay(archive.DirectoryPath);
            Check(
                CapturedRunFiles.Names(archive.DirectoryPath, "native-batch-", "-graph.ndjson").Length == 4,
                "Successful retry overwrote failed graph evidence");

            var path = Path.Combine(archive.DirectoryPath, "native-batch-000128-032-attempt-01-graph.ndjson");
            var preserved =
                archive.WriteRecords(
                    Path.GetFileName(path),
                    new object[] { new { oversized = true } },
                    maximumBytes: 1);

            Check(
                preserved.PreservedPrevious && new FileInfo(path).Length > 0,
                "Budget exhaustion erased valid prior graph evidence");

            archive.WriteRecords("complete.ndjson", Enumerable.Range(0, 10).Select(i => (object)new { i }));
            var partial =
                archive.WriteRecords(
                    "complete.ndjson",
                    Enumerable.Range(0, 10).Select(i => (object)new { i }),
                    maximumBytes: 13);

            Check(
                partial.Truncated
                    && partial.PreservedPrevious
                    && File.ReadLines(Path.Combine(archive.DirectoryPath, "complete.ndjson")).Count() == 10,
                "Partial truncation replaced a complete artifact");

            var exact = archive.WriteRecords("exact-ids.ndjson", new object[] { new { node = long.MinValue + 123 } });
            Check(
                JObject.Parse(File.ReadAllText(Path.Combine(archive.DirectoryPath, exact.File)))["node"]!.Type
                    == JTokenType.String,
                "Int64 identity lost its exact JSON representation");

            var completeChunks = archive.WriteRecordChunks("preserved-chunks.ndjson", records.Take(10));
            var completeManifest = File.ReadAllText(Path.Combine(archive.DirectoryPath, completeChunks.Manifest));
            Replay(archive.DirectoryPath);
            archive.WriteRecords(
                "native-batch-000256-032-attempt-00-expected.ndjson",
                new object[] {
                    new {
                        kind = "expected-course",
                        prefab = "stock",
                        StartCenterBudget = 18,
                        EndCenterBudget = 18,
                        section = DiagnosticArtifacts.NetworkRecords(new[] { segment }).First()
                        }
                    });

            archive.WriteRecords(
                "native-batch-000256-032-attempt-00-owners.ndjson",
                new object[] { new { source = "Road:1", entity = "committed" } });

            archive.WriteRecords(
                "native-batch-000256-032-attempt-00-graph.ndjson",
                new object[] { new { oversized = true } },
                maximumBytes: 1);

            archive.Write(Array.Empty<string>(), new Dictionary<string, double>(), null);
            Refuses(
                () => Replay(archive.DirectoryPath),
                "Incomplete batch artifact",
                "Truncated captured graph may not establish coverage");

            var filler = new { payload = new string('x', 32768) };

            for (var i = 0; i < 18; i++)
                archive.WriteRecords($"budget-{i}.ndjson", Enumerable.Repeat((object)filler, 300));

            archive.WriteRecords("budget-tail.ndjson", Enumerable.Repeat((object)new { i = 0 }, 100000));
            var failedChunks = archive.WriteRecordChunks("preserved-chunks.ndjson", records.Take(10));
            Check(
                failedChunks.Truncated
                    && failedChunks.PreservedPrevious
                    && File.ReadAllText(Path.Combine(archive.DirectoryPath, completeChunks.Manifest))
                        == completeManifest,
                "Failed recapture replaced a complete chunk manifest");

            var finalRecord = new object[] { new { payload = new string('y', 65536) } };
            Check(
                archive.WriteRecords("ordinary-full.ndjson", finalRecord).Truncated,
                "Ordinary-budget exhaustion fixture did not fill its budget");

            Check(
                archive.WriteRecords("reserved-final.ndjson", finalRecord, final: true).Records == 1,
                "Early transcripts consumed final-readback reserve");
        }
        finally
        {
            if (Directory.Exists(root))
                Directory.Delete(root, true);
        }

        Console.WriteLine(
            "PASS: synthetic diagnostic manifest/owner/graph roundtrip retains rejected and accepted attempts independently; truncated evidence cannot claim coverage.");
    }
}
