using Game.Prefabs;
using ImportEarthLocation.Core;
using ImportEarthLocation.Integration;
using Unity.Entities;

static class Program
{
    static NetPiecePrefab Piece(params float[] offsets)
    {
        var lane = new NetLanePrefab();
        lane.Components[typeof(TrackLane)] = new TrackLane { m_TrackType = Game.Net.TrackTypes.Train };
        var piece = new NetPiecePrefab();
        piece.Components[typeof(NetPieceLanes)] = new NetPieceLanes
        {
            m_Lanes =
                offsets.Select(x => new NetLaneInfo { m_Lane = lane, m_Position = new Position { x = x } }).ToArray()
        };

        return piece;
    }

    static (PrefabResolver Resolver, ImportLog Log, TrackPrefab Prefab) Setup(
        string name,
        params NetPiecePrefab[] pieces)
    {
        var section =
            new NetSectionPrefab
            {
                m_Pieces = pieces.Select(otherPrefab => new NetPieceInfo { m_Piece = otherPrefab }).ToArray()
            };

        var prefab = new TrackPrefab
        {
            name = name,
            Entity = new Entity(1),
            m_TrackType = Game.Net.TrackTypes.Train,
            m_Sections = new[] { new NetSectionInfo { m_Section = section } }
        };

        var system = new PrefabSystem();
        system.Catalog.Add(prefab);
        system.EntityManager.Components[prefab.Entity] =
            new NetGeometryData { m_DefaultWidth = 8, m_ElevatedWidth = 8 };

        var log = new ImportLog();

        return (new PrefabResolver(system, log), log, prefab);
    }

    static int Main()
    {
        try
        {
            Run();
            return 0;
        }
        catch (Exception exception)
        {
            Console.Error.WriteLine(exception);
            return 1;
        }
    }

    static void Run()
    {
        var single = Setup("Twoway Train Track", Piece(0, 0), Piece(0));
        Check(!single.Resolver.CanResolveRail(out _), "Missing parent buffer rejected valid child lanes");
        Check(
            single.Log.Messages.Any(
                text => text.Contains("physicalTracks=1") && text.Contains("prefab-section-piece-lanes")),
            "Child lanes not read back");

        Check(
            !single.Log.Messages.Any(text => text.Contains("namedFallback=True")),
            "Verified child lanes used fallback");

        var doubleTrack = Setup("Twoway Train Track", Piece(-2, 2));
        Check(doubleTrack.Resolver.CanResolveRail(out _), "Two physical tracks accepted");
        var split = Setup("Twoway Train Track");
        var sharedSection = new NetSectionPrefab { m_Pieces = new[] { new NetPieceInfo { m_Piece = Piece(0, 0) } } };
        split.Prefab.m_Sections =
            new[] {
                new NetSectionInfo { m_Section = sharedSection, m_Offset = new Position { x = -2 } },
                new NetSectionInfo { m_Section = sharedSection, m_Offset = new Position { x = 2 } }
                };

        Check(
            split.Resolver.CanResolveRail(out _)
                && split.Log.Messages.Any(text => text.Contains("selected=Twoway Train Track; physicalTracks=2")),
            "Parallel section offsets/reused section asset were not counted as two physical tracks");

        var unknown = Setup("Twoway Train Track");
        Check(!unknown.Resolver.CanResolveRail(out _), "Exact vanilla fallback unavailable");
        Check(
            unknown.Log.Messages.Any(text => text.Contains("nativeLaneReadback=False")),
            "Unknown readback claimed verified");

        var nested = Setup("Twoway Train Track");
        var section = nested.Prefab.m_Sections![0].m_Section;
        var child = new NetSectionPrefab { m_Pieces = new[] { new NetPieceInfo { m_Piece = Piece(0) } } };
        section.m_SubSections = new[] { new NetSubSectionInfo { m_Section = child } };
        child.m_SubSections = new[] { new NetSubSectionInfo { m_Section = section } };
        Check(!nested.Resolver.CanResolveRail(out _), "Recursive subsection/cycle handling failed");
        var nativeDouble = Setup("Double Train Track", Piece(0));
        nativeDouble.Prefab.Entity = new Entity(1);
        // Reproduce installed 12 m double-track asset with coincident section-local
        // lane coordinates, unlike the 8 m single bidirectional corridor.
        var doubleSystem = new PrefabSystem();
        doubleSystem.Catalog.Add(nativeDouble.Prefab);
        doubleSystem.EntityManager.Components[nativeDouble.Prefab.Entity] =
            new NetGeometryData { m_DefaultWidth = 12, m_ElevatedWidth = 12 };

        var fallbackLog = new ImportLog();
        Check(
            new PrefabResolver(doubleSystem, fallbackLog).CanResolveRail(out _),
            "Installed Double Train Track falsely rejected by section-local deduplication");

        Check(
            fallbackLog.Messages.Any(text => text.Contains("physicalCountVerified=False")),
            "Layout fallback claimed composed track count verified");

        foreach (var name in new[] { "Twoway Train Track - Station Side", "LiftBridge01", "Custom Train Track" })
            Check(!Setup(name, Piece(0)).Resolver.CanResolveRail(out _), "Special prefab accepted: " + name);

        var absent = new PrefabResolver(new PrefabSystem(), new ImportLog());
        Check(
            !absent.CanResolveRail(out var reason) && reason.Contains("affected railway definitions are unavailable"),
            "Missing rail did not clearly report the affected native contract");

        var roads = new PrefabSystem();

        foreach (var lanes in new[] { 1, 2, 3, 4, 5 })
        {
            var road =
                new RoadPrefab
                {
                    name = $"Highway Oneway - {lanes} lane{(lanes == 1 ? "" : "s")}",
                    Entity = new Entity(lanes),
                    m_HighwayRules = true
                };

            roads.Catalog.Add(road);
            roads.EntityManager.TypedComponents[(road.Entity, typeof(NetGeometryData))] =
                new NetGeometryData { m_DefaultWidth = lanes * 4 + 4, m_ElevatedWidth = lanes * 4 + 4 };

            roads.EntityManager.TypedComponents[(road.Entity, typeof(RoadData))] =
                new RoadData { m_Flags = RoadFlags.DefaultIsForward };
        }

        var roadResolver = new PrefabResolver(roads, new ImportLog());
        roads.EntityManager.Lanes[new Entity(3)] = new List<DefaultNetLane>();

        foreach (var offset in new[] { -4f, 0f, 4f })
        {
            var laneEntity = new Entity(100 + (int)offset);
            roads.EntityManager.TypedComponents[(laneEntity, typeof(CarLaneData))] = new CarLaneData();
            roads.EntityManager.Lanes[new Entity(
                3)].Add(new DefaultNetLane { m_Lane = laneEntity, m_Position = new Position { x = offset } });
        }

        roadResolver.CaptureMinimumWidths();
        Check(
            roadResolver.LaneOffsets["highway:True:False:False:3"].SequenceEqual(new[] { -4f, 0f, 4f }),
            "Ramp lane positions were estimated instead of captured from the selected native prefab");

        var highway =
            new PreparedSegment(
                90,
                1,
                2,
                null,
                null,
                NetworkKind.Road,
                "motorway",
                3,
                true,
                false,
                false,
                0)
            { MaximumNativeWidth = 8 };

        Check(roadResolver.Resolve(highway) == new Entity(3), "Narrow estimate silently downgraded a 3 lane mainline");
        Check(
            roadResolver.Resolve(highway with { Lanes = 4 }) == new Entity(4),
            "Available four-lane highway was capped or width-downgraded");

        Check(
            roadResolver.Resolve(highway with { Lanes = 5 }) == new Entity(5),
            "Available five-lane highway was capped or width-downgraded");

        Check(
            roadResolver.Resolve(highway with { Lanes = 6 }) == new Entity(5),
            "Highway supported limit must be five lanes");

        Check(roadResolver.Resolve(highway with { Lanes = 1 }) == new Entity(1), "Explicit one-lane ramp was widened");
        // Latest live failure: VA 123 way 227895247 contains six one-way lanes.
        // The old snapshot clamped it to five (16 m), while catalog lookup chose 24 m.
        var ordinary = new PrefabSystem();

        foreach (var entry in new[] {
            (name: "Small Road Oneway - 3 lanes", id: 16, width: 16f),
            (name: "Medium Road Oneway", id: 24, width: 24f)
            })
        {
            var road = new RoadPrefab { name = entry.name, Entity = new Entity(entry.id) };
            ordinary.Catalog.Add(road);
            ordinary.EntityManager.TypedComponents[(road.Entity, typeof(NetGeometryData))] =
                new NetGeometryData { m_DefaultWidth = entry.width, m_ElevatedWidth = entry.width + 1 };

            ordinary.EntityManager.TypedComponents[(road.Entity, typeof(RoadData))] =
                new RoadData { m_Flags = RoadFlags.DefaultIsForward };
        }

        var ordinaryResolver = new PrefabResolver(ordinary, new ImportLog());
        var widths = ordinaryResolver.CaptureMinimumWidths();
        var chainBridge =
            highway with
            {
                WayId = 227895247,
                Classification = "primary",
                Lanes = 6,
                NativeWidth = 16,
                MaximumNativeWidth = 15.775f
            };

        // Regular one-way roads come in one to seven lanes. Six lanes keeps its own
        // snapshot key; with the six-lane asset missing from this catalog, the
        // nearest present one (the four-lane "Medium Road Oneway") is used, and
        // snapshot, actual asset, widths and direction must agree on it.
        Check(
            PrefabResolver.SizedPaletteKey(chainBridge).EndsWith(":6"),
            "Regular one-way stock representation does not keep six lanes");

        Check(
            widths[PrefabResolver.SizedPaletteKey(chainBridge)] == 24,
            "Preparation snapshot has wrong native footprint for the nearest regular one-way representation");

        var prepared = PrefabResolver.ApplyMetadata(chainBridge, widths, ordinaryResolver.LaneOffsets);
        Check(
            prepared.NativeWidth == 24 && prepared.Lanes == 6,
            "Metadata refresh changes source lane evidence or audits the wrong selected footprint");

        Check(
            ordinaryResolver.Resolve(chainBridge) == new Entity(24),
            "Narrow corridor prevented the nearest regular one-way representation");

        Check(
            ordinaryResolver.Resolve(chainBridge with { MaximumNativeWidth = float.PositiveInfinity })
                == new Entity(24),
            "Spacing estimate changes catalog asset selection");

        Check(
            ordinaryResolver.ResolveMetadata(chainBridge).NativeWidth == 24,
            "Actual metadata readback does not match preparation snapshot");

        Check(
            ordinaryResolver.ResolveMetadata(chainBridge with { Bridge = true }).NativeWidth == 25,
            "Elevated width omitted from the audited footprint");

        Check(
            ordinaryResolver.Resolve(chainBridge with { OneWay = false }) == Entity.Null,
            "Estimated-width relaxation changed required road direction");

        Check(
            ordinaryResolver.Resolve(chainBridge with { Classification = "PRIMARY", Lanes = 2 }) == new Entity(16),
            "Case or absent two-lane exact name prevented ordinary one-way catalog fallback");

        Check(
            ordinaryResolver.ResolveMetadata(chainBridge with { Classification = "primary", Lanes = 2 }).NativeWidth
                == 16,
            "Small spacing deficit still rejects a usable native road");

        var root = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "../../../../../"));
        var ui =
            new SourceText(
                File.ReadAllText(Path.Combine(root, "ImportEarthLocation/Systems/ImportEarthLocationUISystem.cs")));

        var apply =
            new SourceText(File.ReadAllText(Path.Combine(root, "ImportEarthLocation/Systems/ImportApplySystem.cs")));

        Check(
            ui.Contains("CanResolveRail(out var railReason)") && ui.Contains("_log.Warn(railReason)"),
            "Unavailable rail must be reported without aborting independent roads");

        Check(
            apply.Contains("Native entity/ownership safety still applies")
                && apply.Contains("NetworkGeometrySafety.Validate"),
            "Native safety/geometry-quality distinction missing");

        var placement =
            new SourceText(
                File.ReadAllText(Path.Combine(root, "ImportEarthLocation/Systems/NativeNetworkPlacement.cs")));

        Check(!placement.Contains("Lanes=segment.OneWay?1:2"), "Placement fallback still rewrites source lane counts");
        Check(
            !placement.Contains("Major network placement failure"),
            "Partial source failures still discard every valid native definition");

        Check(
            placement.Contains("Native metadata unavailable for")
                && placement.Contains("supported sections retained for generation"),
            "Unsupported native contracts must be visible and scoped to affected sections");

        Check(
            apply.Contains("RemainingNetworks") && apply.Contains("Old native network teardown did not settle"),
            "Old-network retirement guard missing");

        Check(
            ui.IndexOf("NativeAssets=CaptureNativeAssets(result.Segments)", StringComparison.Ordinal)
                < ui.IndexOf("_processTask=FinalizeNetworkDesignAsync", StringComparison.Ordinal)
                && ui.Contains("DesignPasses>=1")
                && ui.Contains("NetworkTerrain=networkTerrain"),
            "Native snapshot must precede the single design, from pre-roadbed terrain");

        Check(
            placement.Contains("pending batch rolled back") && placement.Contains("continuing other sections"),
            "Definition failure must clean its pending work and report unavailable sections");

        var commit =
            new SourceText(File.ReadAllText(Path.Combine(root, "ImportEarthLocation/Systems/NetworkCommitSystem.cs")));

        Check(
            commit.Contains("NativePreviewSafety.RequireEdge")
                && commit.Contains("catch(NativeSafetyException")
                && commit.Contains("IssueOwnedClear"),
            "Unsafe native contracts must not be downgraded into quality warnings");

        Console.WriteLine(
            "PASS: production resolver with offline native stand-ins: double-track fallback, strict direction/assets, highway 1-5 lanes, six-lane VA 123 footprint, ordinary one-way fallback, elevated widths, native lane positions; bounded reconciliation and atomic replacement guards.");
    }
}

// Source guards compare code without its layout, so that formatting cannot break them.
sealed class SourceText
{
    private readonly string _compact;

    public SourceText(string text) => _compact = Compact(text);

    public bool Contains(string text) => _compact.Contains(Compact(text));

    public int IndexOf(string text, StringComparison comparison) => _compact.IndexOf(Compact(text), comparison);

    private static string Compact(string text) => string.Concat(text.Where(character => !char.IsWhiteSpace(character)));
}
