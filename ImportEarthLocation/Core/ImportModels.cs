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

public sealed record ImportRequest(
    double Latitude,
    double Longitude,
    MapSizeProfile MapSize,
    bool IncludeSecondaryRoads)
{
    // Legacy constructor flag migrates to the new tier; rail remains independently enabled.
    public RoadImportLevel RoadLevel { get; init; } = IncludeSecondaryRoads ? RoadImportLevel.Secondary : RoadImportLevel.Primary;
    public bool ImportRail { get; init; } = true;
    public long MapSessionId { get; init; }
    public long Generation { get; init; }
    public bool ApplyGroundSurfaces { get; init; }
    public bool CreateAirConnections { get; init; }
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
public sealed record OsmRelation(long Id, IReadOnlyList<(long wayId, bool inner)> Members, IReadOnlyDictionary<string, string> Tags);

public enum NetworkKind
{
    Road,
    Rail
}

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
    public float? ExplicitIncline { get; init; }
    public long CorridorId { get; init; }
    public float MaximumNativeWidth { get; init; } = float.PositiveInfinity;
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
}

public sealed class TerrainTilePixels
{
    public int Zoom { get; init; }
    public int X { get; init; }
    public int Y { get; init; }
    public UnityEngine.Color32[] Pixels { get; init; } = Array.Empty<UnityEngine.Color32>();
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
    public SurfaceMask? SurfaceMask { get; init; }
    public IReadOnlyList<AirDestination> AirDestinations { get; init; } = Array.Empty<AirDestination>();
    public ClimateNormals? Climate { get; init; }
    public WaterPlan? Water { get; init; }
    public string FeatureReport { get; set; } = "";
}

public sealed record ClimateNormals(float[] Temperatures, float[] Precipitation, float[] WindSpeed, float[] WindFrom,
    float SourceElevation, string SourceVersion);

public enum GroundMaterial : byte { Base, Sand, DryGrass, Grass, Dirt }
public enum SurfaceTransition : byte { Hard, Medium, Soft }
public sealed record SurfaceMask(int Resolution, double WidthMeters, byte[] Material, byte[] Intensity, string Report)
{
    public byte[] Confidence { get; init; } = Array.Empty<byte>();
    public SurfaceTransition[] Transitions { get; init; } = Array.Empty<SurfaceTransition>();
}
public sealed record AirDestination(string Name, double DistanceKm, double Bearing, float3 Position);
public sealed record WaterPlan(bool Coastal, int Rivers, int Lakes, int Skipped, string Report);

public sealed record ImportSummary(
    int RoadSegments,
    int RailSegments,
    int SkippedSegments,
    float MinimumElevation,
    float MaximumElevation,
    IReadOnlyList<string> Warnings);
