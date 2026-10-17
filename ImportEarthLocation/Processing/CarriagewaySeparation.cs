using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Existing callers use the same authoritative pairing owner as final design.
public static class CarriagewaySeparation
{
    public static IReadOnlyList<PreparedSegment> Repair(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
        =>NetworkPairedCorridors.Prepare(source,log,token);
}
