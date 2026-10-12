using System;
using Game.Simulation;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Integration;

// Terrain bounds are sample centres; outside connections use nominal corners.
public static class NativeMapBoundary
{
    public static bool TryGetHalf(TerrainSystem terrain,double width,int sampleCount,ImportLog log,out float half)
    {
        half=(float)(width*.5);var data=terrain.GetHeightData(false);
        if(!data.heights.IsCreated||!math.isfinite(half)||half<=0)return false;
        var bounds=TerrainUtils.GetBounds(ref data);var resolution=(int)Math.Sqrt(sampleCount);
        var tolerance=Math.Max(2,width/Math.Max(1,resolution-1));
        var matches=math.all(math.isfinite(bounds.min))&&math.all(math.isfinite(bounds.max))&&
            math.abs(bounds.min.x+half)<=tolerance&&math.abs(bounds.max.x-half)<=tolerance&&
            math.abs(bounds.min.z+half)<=tolerance&&math.abs(bounds.max.z-half)<=tolerance;
        log.Diagnostic($"MAP BOUNDARY nominalHalf={half:R}; sampleMin={bounds.min}; sampleMax={bounds.max}; toleranceMeters={tolerance:R}; registrationMatches={matches}; outsideObjectsUseNominalBoundary=true");
        return matches;
    }
}
