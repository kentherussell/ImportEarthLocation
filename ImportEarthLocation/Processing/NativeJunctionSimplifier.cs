using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Only remap-created return loops inside an explicit consolidated junction.
// No proximity joining, real branch contraction, rail or roundabout deletion.
public static class NativeJunctionSimplifier
{
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var incident = new Dictionary<long,List<PreparedSegment>>();
        foreach (var s in source.Where(s=>s.Kind==NetworkKind.Road))
            foreach (var id in new[]{s.StartNodeId,s.EndNodeId})
            { if(!incident.TryGetValue(id,out var arms))incident[id]=arms=new List<PreparedSegment>(); arms.Add(s); }
        var removed = new HashSet<PreparedSegment>();
        foreach (var a in source)
        {
            token.ThrowIfCancellationRequested();
            if(removed.Contains(a)||!Eligible(a)||!a.OneWay||a.StartNodeId==a.EndNodeId)continue;
            if(!incident.TryGetValue(a.EndNodeId,out var atEnd)||atEnd.Count!=2)continue;
            var b=atEnd.FirstOrDefault(s=>s!=a);
            if(b==null||!Eligible(b)||b.StartNodeId!=a.EndNodeId||b.EndNodeId!=a.StartNodeId||b.WayId!=a.WayId)continue;
            if(a.Classification!=b.Classification||a.Layer!=b.Layer||a.Lanes!=b.Lanes||a.OneWay!=b.OneWay||
                a.NativeWidth!=b.NativeWidth||a.SourceSpeedKph!=b.SourceSpeedKph||a.ExplicitIncline!=b.ExplicitIncline||
                !a.SourceTags.OrderBy(p=>p.Key).SequenceEqual(b.SourceTags.OrderBy(p=>p.Key))||
                !a.RawSourceTags.OrderBy(p=>p.Key).SequenceEqual(b.RawSourceTags.OrderBy(p=>p.Key)))continue;
            if(!incident.TryGetValue(a.StartNodeId,out var anchorArms)||!anchorArms.Any(s=>s.ConsolidatedAvenue))continue;
            if(anchorArms.Any(s=>!Eligible(s)||s.Layer!=a.Layer))continue;
            // Original endpoint no longer equals the remapped avenue center.
            if(a.OriginalGeometry.Count<2||b.OriginalGeometry.Count<2||
                math.distance(a.OriginalGeometry[0].xz,a.Start.xz)<.5f||
                math.distance(b.OriginalGeometry[b.OriginalGeometry.Count-1].xz,b.End.xz)<.5f||
                math.distance(a.OriginalGeometry[0].xz,b.OriginalGeometry[b.OriginalGeometry.Count-1].xz)<=.5f||
                math.distance(a.OriginalGeometry[a.OriginalGeometry.Count-1].xz,b.OriginalGeometry[0].xz)>.05f)continue;
            var bound=Math.Min(12,Math.Max(4,(a.NativeWidth>0?a.NativeWidth*.5f:8)+2));
            if(NetworkVerticalProfiles.ArcLength(a)+NetworkVerticalProfiles.ArcLength(b)>24)continue;
            var inside=true;
            foreach(var s in new[]{a,b})for(var i=0;i<=16;i++)
                if(math.distance(NetworkNormalizer.Evaluate(s,i/16f).xz,a.Start.xz)>bound)inside=false;
            if(!inside)continue;
            removed.Add(a);removed.Add(b);
            log.Diagnostic($"NATIVE MICROCLUSTER normalized=true; way={a.WayId}; retainedAnchor={a.StartNodeId}; removedInterior={a.EndNodeId}; spans=2; interiorDegree=2; radiusBoundMeters={bound}; originalRemapEvidence=true; realBranchesRemoved=0; regradePending=true");
        }
        log.Count("Remap-created return loop sections normalized",removed.Count);
        if(removed.Count>0)log.DiagnosticRecords("native-normalized-return-loops.ndjson",DiagnosticArtifacts.NetworkRecords(source.Where(s=>removed.Contains(s)).ToArray()));
        return removed.Count==0?source:source.Where(s=>!removed.Contains(s)).ToArray();
    }
    private static bool Eligible(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.Bridge&&!s.Tunnel&&!s.Roundabout&&
        !s.StartTerminal&&!s.EndTerminal&&!s.StartMiniRoundabout&&!s.EndMiniRoundabout&&
        s.StartRoundaboutRadius==0&&s.EndRoundaboutRadius==0&&s.ExplicitCutSides==0&&s.ExplicitWallSides==0&&s.RetainingWallSides==0;
}
