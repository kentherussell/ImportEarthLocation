using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// A source port preserves native topology/provenance, not necessarily a bend
// in the vertical design. Callers must additionally check incidence and fixed
// native height/slope ownership before sharing a profile through that port.
public static class NetworkBoundaryPolicy
{
    public static bool EndpointMarker(PreparedSegment segment, bool start) => start ?
        segment.StartTerminal
            || segment.StartLevelCrossing
            || segment.StartMiniRoundabout
            || segment.StartRoundaboutRadius != 0
            :
        segment.EndTerminal
            || segment.EndLevelCrossing
            || segment.EndMiniRoundabout
            || segment.EndRoundaboutRadius != 0;
}
