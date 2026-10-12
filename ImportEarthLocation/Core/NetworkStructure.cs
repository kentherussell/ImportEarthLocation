namespace ImportEarthLocation.Core;

public static class NetworkStructure
{
    // OSM layer is relative ordering, not a height in metres.
    public static float Offset(PreparedSegment segment) => segment.Bridge ? 5f : segment.Tunnel ? -8f : 0f;
    public static bool IsStructure(PreparedSegment segment) => segment.Bridge || segment.Tunnel;
}
