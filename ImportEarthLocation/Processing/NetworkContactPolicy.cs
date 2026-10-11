using System;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public enum NetworkContactKind { Junction, LevelCrossing, IndependentLevels, HorizontalConflict, UnconnectedSurfaceCrossing }

// Shared by horizontal spacing and vertical safety. Only evidence of separate
// levels is allowed to create a clearance constraint; proximity is not a junction.
public static class NetworkContactPolicy
{
    public static NetworkContactKind Classify(PreparedSegment a,PreparedSegment b,float3 p,float3 q,float2 headingA,float2 headingB,bool localJunction)
    {
        if(localJunction&&a.Kind==b.Kind)return NetworkContactKind.Junction;
        var shared=a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId;
        var nearShared=shared&&(math.distance(p.xz,a.Start.xz)<30||math.distance(p.xz,a.End.xz)<30)&&
            (math.distance(q.xz,b.Start.xz)<30||math.distance(q.xz,b.End.xz)<30);
        if(a.Kind!=b.Kind&&nearShared&&!a.Bridge&&!b.Bridge&&!a.Tunnel&&!b.Tunnel&&a.Layer==b.Layer)
            return NetworkContactKind.LevelCrossing;
        var gap=Math.Abs(p.y-q.y);
        if(a.Layer!=b.Layer||a.Bridge!=b.Bridge||a.Tunnel!=b.Tunnel||gap>=NetworkGeometrySafety.DeckGap)
            return NetworkContactKind.IndependentLevels;
        var parallel=Math.Abs(math.dot(math.normalizesafe(headingA),math.normalizesafe(headingB)))>.8f;
        return parallel?NetworkContactKind.HorizontalConflict:NetworkContactKind.UnconnectedSurfaceCrossing;
    }
}
