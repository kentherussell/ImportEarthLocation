using System.Collections.Generic;
using System.IO;
using System.Linq;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Gameplay-first topology. Never join unrelated levels or substitute nearby
// roads for actual source connections.
public static class StarterNetworkPolicy
{
    public static IReadOnlyList<PreparedSegment> WithoutSideStructures(IReadOnlyList<PreparedSegment> source)
        => source
            .Select(
                segment => segment.ExplicitWallSides == NetworkSides.None
                    && segment.RetainingWallSides == NetworkSides.None
                    && segment.QuaySides == NetworkSides.None
                    ? segment
                    :
            segment with
            {
                ExplicitWallSides = NetworkSides.None,
                RetainingWallSides = NetworkSides.None,
                QuaySides = NetworkSides.None
            })
                .ToArray();
}
