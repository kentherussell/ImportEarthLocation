using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace ImportEarthLocation.Core;

public enum ImportStage
{
    Idle,
    Validating,
    DownloadingTerrain,
    DownloadingOsm,
    Processing,
    ReadyForConfirmation,
    Applying,
    Complete,
    Failed
}

public enum PreparationDataIssue { ElevationUnavailable, NetworkIncomplete, ResourceLimit, GeometryUnavailable }

/// <summary>An exhausted bounded data-recovery path, not user input or a software fault.</summary>
public sealed class PreparationDataException : Exception
{
    public PreparationDataIssue Issue { get; }
    public PreparationDataException(PreparationDataIssue issue,string detail,Exception? inner=null) : base(detail,inner) => Issue=issue;
    public string UserMessage => Issue switch
    {
        PreparationDataIssue.ElevationUnavailable => "Elevation data could not be recovered safely. Your map is unchanged; cached downloads are retained. Please retry preparation later.",
        PreparationDataIssue.NetworkIncomplete => "Complete road and railway data is not available yet. Your map is unchanged; cached downloads are retained. Please retry preparation.",
        PreparationDataIssue.GeometryUnavailable => "The available terrain and network geometry could not be reconciled safely within the recovery limits. Your map is unchanged; cached data and diagnostics are retained. See the log for the affected source ways.",
        _ => "Preparation reached a safe memory limit. Your map is unchanged; cached downloads are retained. Close other applications or use a smaller area, then retry."
    };
}

public sealed record ImportRequest(
    double Latitude,
    double Longitude,
    MapSizeProfile MapSize,
    bool IncludeSecondaryRoads)
{
    // Legacy constructor flag migrates to the new tier; rail remains independently enabled.
    public RoadImportLevel RoadLevel { get; init; } = RoadImportLevel.None;
    public bool ImportRail { get; init; } = true;
    public bool ImportForests { get; init; } = true;
    public long MapSessionId { get; init; }
    public long Generation { get; init; }
    public bool ApplyGroundSurfaces { get; init; }
    public bool CreateAirConnections { get; init; }
    public bool CreateSeaConnections { get; init; }
    public bool DiagnosticsEnabled { get; init; }
}

public sealed record MapSizeProfile(string Id, string Label, double WidthMeters, bool RequiresMapExt)
{
    public static readonly MapSizeProfile Vanilla = new("vanilla", "14.336 km (Vanilla)", 14_336d, false);
    public static readonly MapSizeProfile MapExt28 = new("mapext28", "28.672 km (MapExt)", 28_672d, true);
    public static readonly MapSizeProfile MapExt57 = new("mapext57", "57.344 km (MapExt)", 57_344d, true);

    public static MapSizeProfile FromId(string? id) => id switch
    {
        "mapext28" => MapExt28,
        "mapext57" => MapExt57,
        _ => Vanilla
    };
}

public readonly record struct GeoBounds(double South, double West, double North, double East)
{
    public bool CrossesDateline => West < -180d || East > 180d;
}

public readonly record struct OsmNode(long Id, double Latitude, double Longitude);

public sealed record OsmWay(long Id, long[] NodeIds, IReadOnlyDictionary<string, string> Tags);
public readonly record struct OsmMember(string Type, long Id, string Role);
public sealed record OsmRelation(long Id, IReadOnlyList<(long wayId, bool inner)> Members, IReadOnlyDictionary<string, string> Tags)
{
    // Legacy polygon view remains available. Network consumers use exact ordered
    // members, including node-via restrictions and multi-way via paths.
    public IReadOnlyList<OsmMember> OrderedMembers { get; init; } = Array.Empty<OsmMember>();
}

public enum NetworkKind
{
    Road,
    Rail
}

[Flags]
public enum NetworkSides { None = 0, Left = 1, Right = 2, Both = Left | Right }

public sealed record PreparedSegment(
    long WayId,
    long StartNodeId,
    long EndNodeId,
    float3 Start,
    float3 End,
    NetworkKind Kind,
    string Classification,
    int Lanes,
    bool OneWay,
    bool Bridge,
    bool Tunnel,
    int Layer)
{
    public float3? Control1 { get; init; }
    public float3? Control2 { get; init; }
    public IReadOnlyList<float3> SourceGeometry { get; init; } = Array.Empty<float3>();
    public string Name { get; init; } = "";
    public string Reference { get; init; } = "";
    public string RailService { get; init; } = "";
    public IReadOnlyList<long> SourceWayIds { get; init; } = Array.Empty<long>();
    public IReadOnlyList<float3> OriginalGeometry { get; init; } = Array.Empty<float3>();
    public IReadOnlyDictionary<string,string> SourceTags { get; init; } = new Dictionary<string,string>();
    public IReadOnlyDictionary<string,string> RawSourceTags { get; init; } = new Dictionary<string,string>();
    public bool StartMiniRoundabout { get; init; }
    public bool EndMiniRoundabout { get; init; }
    public bool StartTerminal { get; init; }
    public bool EndTerminal { get; init; }
    public int PhysicalTracks { get; init; }
    public int BothWaysLanes { get; init; }
    public float? SourceWidthMeters { get; init; }
    public float? SourceSpeedKph { get; init; }
    public bool ExplicitLanes { get; init; }
    public int ForwardLanes { get; init; }
    public int BackwardLanes { get; init; }
    public bool Roundabout { get; init; }
    public float StartRoundaboutRadius { get; init; }
    public float EndRoundaboutRadius { get; init; }
    // Sides are relative to the prepared segment direction, not compass bearings.
    public NetworkSides ExplicitCutSides { get; init; }
    public NetworkSides ExplicitWallSides { get; init; }
    public NetworkSides RetainingWallSides { get; init; }
    public NetworkSides QuaySides { get; init; }
    public float? ExplicitIncline { get; init; }
    public long CorridorId { get; init; }
    public bool ConsolidatedAvenue { get; init; }
    public float MaximumNativeWidth { get; init; } = float.PositiveInfinity;
    public float NativeWidth { get; init; }
    // Native curves can meet a lane off the shared node's axis. The node anchor
    // stays exact for topology; these offsets affect the deck, not connectivity.
    public float3 StartCurveOffset { get; init; }
    public float3 EndCurveOffset { get; init; }
    public float3 CourseStart => Start + StartCurveOffset;
    public float3 CourseEnd => End + EndCurveOffset;
    public float3 CourseControl1 => (Control1 ?? Unity.Mathematics.math.lerp(Start,End,1f/3)) + StartCurveOffset;
    public float3 CourseControl2 => (Control2 ?? Unity.Mathematics.math.lerp(Start,End,2f/3)) + EndCurveOffset;
    public IReadOnlyList<float> NativeLaneOffsets { get; init; } = Array.Empty<float>();
    public float? StartElevation { get; init; }
    public float? EndElevation { get; init; }
    public PreparedSegment? HorizontalParent { get; init; }
    public float ParentStartT { get; init; }
    public float ParentEndT { get; init; } = 1;
    public bool ProtectedLink => Kind == NetworkKind.Road && Classification.EndsWith("_link", StringComparison.OrdinalIgnoreCase);
}

public sealed class TerrainTile
{
    public int Zoom { get; init; }
    public int X { get; init; }
    public int Y { get; init; }
    public byte[] PngBytes { get; init; } = Array.Empty<byte>();
    public int RecoveryPass { get; init; }
    public TerrainTile? RecoveryParent { get; init; }
}

public sealed class TerrainTilePixels
{
    public int RecoveryPass { get; init; }
    public int Zoom { get; init; }
    public int X { get; init; }
    public int Y { get; init; }
    public UnityEngine.Color32[] Pixels { get; init; } = Array.Empty<UnityEngine.Color32>();
    public float[]? RecoveredElevations { get; init; }
}

public sealed class PreparedImport
{
    public ImportRequest Request { get; init; } = new(0, 0, MapSizeProfile.Vanilla, false);
    public GeoBounds Bounds { get; init; }
    public ushort[] Heightmap { get; init; } = Array.Empty<ushort>();
    public float HeightScaleMeters { get; init; }
    public float SeaLevelMeters { get; init; }
    public IReadOnlyList<PreparedSegment> Segments { get; init; } = Array.Empty<PreparedSegment>();
    public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();
    public int SourceWayCount { get; init; }
    public IReadOnlyList<float3> Trees { get; init; } = Array.Empty<float3>();
    public IReadOnlyList<VegetationCandidate> Vegetation { get; init; } = Array.Empty<VegetationCandidate>();
    public SurfaceMask? SurfaceMask { get; init; }
    public IReadOnlyList<AirDestination> AirDestinations { get; init; } = Array.Empty<AirDestination>();
    public ClimateNormals? Climate { get; init; }
    public WaterPlan? Water { get; init; }
    public OceanMask? Ocean { get; init; }
    public ImportEarthLocation.Processing.OsmDataset? NetworkEvidence { get; init; }
    public string FeatureReport { get; set; } = "";
}

public sealed record ClimateNormals(float[] Temperatures, float[] Precipitation, float[] WindSpeed, float[] WindFrom,
    float SourceElevation, string SourceVersion);

public enum VegetationForm { Broadleaf, Conifer, Palm, Shrub }
public sealed record VegetationCandidate(float3 Position,VegetationForm Form,uint Seed,long SourceId,string Evidence,string Taxon);

public enum GroundMaterial : byte { Base, Sand, DryGrass, Grass, Dirt }
public enum SurfaceTransition : byte { Hard, Medium, Soft }
public sealed record SurfaceMask(int Resolution, double WidthMeters, byte[] Material, byte[] Intensity, string Report)
{
    public byte[] Confidence { get; init; } = Array.Empty<byte>();
    public SurfaceTransition[] Transitions { get; init; } = Array.Empty<SurfaceTransition>();
}
public sealed record AirDestination(string Name, double DistanceKm, double Bearing, float3 Position);
public sealed record RiverSourcePlan(long WayId, string Name, string Origin, float3 Position, float Radius,
    float Depth, float BankLevel, float3 DownstreamProbe)
{
    public bool BoundaryDriven { get; init; }
    public float? TargetSurfaceLevel { get; init; }
    public IReadOnlyList<float3> ChannelProbes { get; init; } = Array.Empty<float3>();
    public IReadOnlyList<float3> BankProbes { get; init; } = Array.Empty<float3>();
    public long BodyId { get; init; }
    public string BodyKind { get; init; } = "river";
    public float SurfaceOffsetMeters { get; init; }
    // Accepted geographic mask on the immutable map raster, not a widened
    // circular approximation. Final earthworks recovery may edit only these cells.
    public IReadOnlyList<int> BedRepairCells { get; init; } = Array.Empty<int>();
}
public sealed record WaterPlan(bool Coastal, int Rivers, int Lakes, int Skipped, string Report)
{
    public IReadOnlyList<RiverSourcePlan> Sources { get; init; } = Array.Empty<RiverSourcePlan>();
    public float WaterCellSize { get; init; }
    public ushort[]? ConditionedHeightmap { get; init; }
    // Geographic stage evidence is independent of whether a circular regulator fits.
    public IReadOnlyList<WaterAreaStage> AreaStages { get; init; } = Array.Empty<WaterAreaStage>();
    public IReadOnlyList<WaterReachStage> ReachStages { get; init; } = Array.Empty<WaterReachStage>();
    public TerrainEditSet? BedEdits { get; init; }
    public TerrainEditSet? FinalBedEdits { get; init; }
    public ushort[]? PlannedBedHeightmap { get; init; }
    public const string GeometryVersion = "geographic-stage-v2";
}

public sealed record WaterAreaStage(long BodyId, string Kind, float Level);
public sealed record WaterReachStage(long BodyId, long WayId, float3 Start, float3 End,
    float StartLevel, float EndLevel, float HalfWidth, long? PolygonId);
public sealed record TerrainEditSet(string Producer, string Baseline, int Resolution, byte[] Mask);
// 0 known land/barrier, 1 accepted directed ocean, 2 conflicting, 3 unseeded.
public sealed record OceanMask(int Resolution,double WidthMeters,byte[] Classification,string Version);

public sealed record ImportSummary(
    int RoadSegments,
    int RailSegments,
    int SkippedSegments,
    float MinimumElevation,
    float MaximumElevation,
    IReadOnlyList<string> Warnings);
