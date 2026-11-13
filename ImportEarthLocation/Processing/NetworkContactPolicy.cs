using System;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public enum NetworkContactKind
{
    Junction,
    LevelCrossing,
    IndependentLevels,
    HorizontalConflict,
    UnconnectedSurfaceCrossing
}

// Shared by horizontal spacing and vertical safety. Only evidence of separate
// levels is allowed to create a clearance constraint; proximity is not a junction.
public static class NetworkContactPolicy
{
    public static bool OpposingMainlines(
        PreparedSegment firstSegment,
        PreparedSegment secondSegment,
        float2 headingA,
        float2 headingB)
    {
        var related =
            firstSegment.Kind == NetworkKind.Road
                && secondSegment.Kind == NetworkKind.Road
                && firstSegment.OneWay
                && secondSegment.OneWay
                && !firstSegment.ProtectedLink
                && !secondSegment.ProtectedLink
                && !firstSegment.Roundabout
                && !secondSegment.Roundabout
                && !firstSegment.Tunnel
                && !secondSegment.Tunnel
                && firstSegment.Layer == secondSegment.Layer
                && firstSegment.Classification == secondSegment.Classification
                && firstSegment.Classification is "motorway" or "trunk"
                    && (firstSegment.CorridorId != 0 && firstSegment.CorridorId == secondSegment.CorridorId
                || firstSegment.Reference.Length > 0 && firstSegment.Reference == secondSegment.Reference
                || firstSegment.Name.Length > 0 && firstSegment.Name == secondSegment.Name)
                && math.dot(math.normalizesafe(headingA), math.normalizesafe(headingB)) <= -Math.Cos(Math.PI / 12);

        if (!related)
            return false;

        // Require a separating axis for the complete centreline control hulls.
        // Tangent similarity alone could misclassify a shallow real crossing.
        var direction = math.normalizesafe(headingA - headingB);
        var normal = new float2(-direction.y, direction.x);

        (float Min, float Max) Range(PreparedSegment segment)
        {
            var p = math.dot(segment.CourseStart.xz, normal);
            var q = math.dot(segment.CourseControl1.xz, normal);
            var r = math.dot(segment.CourseControl2.xz, normal);
            var t = math.dot(segment.CourseEnd.xz, normal);

            return (Math.Min(Math.Min(p, q), Math.Min(r, t)), Math.Max(Math.Max(p, q), Math.Max(r, t)));
        }

        var firstItem = Range(firstSegment);
        var secondItem = Range(secondSegment);

        return firstItem.Min > secondItem.Max + .05f || secondItem.Min > firstItem.Max + .05f;
    }

    public static NetworkContactKind Classify(
        PreparedSegment firstSegment,
        PreparedSegment secondSegment,
        float3 point,
        float3 otherPoint,
        float2 headingA,
        float2 headingB,
        bool localJunction)
    {
        if (localJunction && firstSegment.Kind == secondSegment.Kind)
            return NetworkContactKind.Junction;

        var shared =
            firstSegment.StartNodeId == secondSegment.StartNodeId
                || firstSegment.StartNodeId == secondSegment.EndNodeId
                || firstSegment.EndNodeId == secondSegment.StartNodeId
                || firstSegment.EndNodeId == secondSegment.EndNodeId;

        var nearShared =
            shared
                && (math.distance(point.xz, firstSegment.Start.xz) < 30
                    || math.distance(point.xz, firstSegment.End.xz) < 30)
                && (math.distance(otherPoint.xz, secondSegment.Start.xz) < 30
                || math.distance(otherPoint.xz, secondSegment.End.xz) < 30);

        if (firstSegment.Kind != secondSegment.Kind
            && nearShared
            && !firstSegment.Bridge
            && !secondSegment.Bridge
            && !firstSegment.Tunnel
            && !secondSegment.Tunnel
            && firstSegment.Layer == secondSegment.Layer)
            return NetworkContactKind.LevelCrossing;

        if (OpposingMainlines(firstSegment, secondSegment, headingA, headingB))
            return NetworkContactKind.HorizontalConflict;

        var gap = Math.Abs(point.y - otherPoint.y);

        if (firstSegment.Layer != secondSegment.Layer
            || firstSegment.Bridge != secondSegment.Bridge
            || firstSegment.Tunnel != secondSegment.Tunnel
            || gap >= NetworkGeometrySafety.DeckGap)
            return NetworkContactKind.IndependentLevels;

        var parallel = Math.Abs(math.dot(math.normalizesafe(headingA), math.normalizesafe(headingB))) > .8f;

        return parallel ? NetworkContactKind.HorizontalConflict : NetworkContactKind.UnconnectedSurfaceCrossing;
    }
}
