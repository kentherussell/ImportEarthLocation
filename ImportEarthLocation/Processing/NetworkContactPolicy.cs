using System;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public enum NetworkContactKind { Junction, LevelCrossing, IndependentLevels, HorizontalConflict, UnconnectedSurfaceCrossing }

// Shared by horizontal spacing and vertical safety. Only evidence of separate
// levels is allowed to create a clearance constraint; proximity is not a junction.
public static class NetworkContactPolicy
{
    public static bool OpposingMainlines(PreparedSegment a,PreparedSegment b,float2 headingA,float2 headingB)
    {
        var related=a.Kind==NetworkKind.Road&&b.Kind==NetworkKind.Road&&a.OneWay&&b.OneWay&&!a.ProtectedLink&&!b.ProtectedLink&&
            !a.Roundabout&&!b.Roundabout&&!a.Tunnel&&!b.Tunnel&&a.Layer==b.Layer&&a.Classification==b.Classification&&
            a.Classification is "motorway" or "trunk"&&
            (a.CorridorId!=0&&a.CorridorId==b.CorridorId||a.Reference.Length>0&&a.Reference==b.Reference||a.Name.Length>0&&a.Name==b.Name)&&
            math.dot(math.normalizesafe(headingA),math.normalizesafe(headingB))<=-Math.Cos(Math.PI/12);
        if(!related)return false;
        // Require a separating axis for the complete centreline control hulls.
        // Tangent similarity alone could misclassify a shallow real crossing.
        var direction=math.normalizesafe(headingA-headingB);var normal=new float2(-direction.y,direction.x);
        (float Min,float Max) Range(PreparedSegment s)
        {
            var p=math.dot(s.CourseStart.xz,normal);var q=math.dot(s.CourseControl1.xz,normal);
            var r=math.dot(s.CourseControl2.xz,normal);var t=math.dot(s.CourseEnd.xz,normal);
            return(Math.Min(Math.Min(p,q),Math.Min(r,t)),Math.Max(Math.Max(p,q),Math.Max(r,t)));
        }
        var x=Range(a);var y=Range(b);
        return x.Min>y.Max+.05f||y.Min>x.Max+.05f;
    }

    public static NetworkContactKind Classify(PreparedSegment a,PreparedSegment b,float3 p,float3 q,float2 headingA,float2 headingB,bool localJunction)
    {
        if(localJunction&&a.Kind==b.Kind)return NetworkContactKind.Junction;
        var shared=a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId;
        var nearShared=shared&&(math.distance(p.xz,a.Start.xz)<30||math.distance(p.xz,a.End.xz)<30)&&
            (math.distance(q.xz,b.Start.xz)<30||math.distance(q.xz,b.End.xz)<30);
        if(a.Kind!=b.Kind&&nearShared&&!a.Bridge&&!b.Bridge&&!a.Tunnel&&!b.Tunnel&&a.Layer==b.Layer)
            return NetworkContactKind.LevelCrossing;
        if(OpposingMainlines(a,b,headingA,headingB))return NetworkContactKind.HorizontalConflict;
        var gap=Math.Abs(p.y-q.y);
        if(a.Layer!=b.Layer||a.Bridge!=b.Bridge||a.Tunnel!=b.Tunnel||gap>=NetworkGeometrySafety.DeckGap)
            return NetworkContactKind.IndependentLevels;
        var parallel=Math.Abs(math.dot(math.normalizesafe(headingA),math.normalizesafe(headingB)))>.8f;
        return parallel?NetworkContactKind.HorizontalConflict:NetworkContactKind.UnconnectedSurfaceCrossing;
    }
}
