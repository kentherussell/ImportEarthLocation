using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Conservative network-only topology repairs. Never moves terrain or merges carriageways.</summary>
public static class NetworkHardener
{
    private static bool SameLevel(PreparedSegment a, PreparedSegment b) => a.Kind == b.Kind && a.Bridge == b.Bridge && a.Tunnel == b.Tunnel && a.Layer == b.Layer;
    private static bool ConflictingNames(PreparedSegment a, PreparedSegment b) =>
        a.Reference.Length > 0 && b.Reference.Length > 0 && !a.Reference.Equals(b.Reference,StringComparison.OrdinalIgnoreCase) ||
        a.Name.Length > 0 && b.Name.Length > 0 && !a.Name.Equals(b.Name,StringComparison.OrdinalIgnoreCase) && !(a.Reference.Length > 0 && a.Reference.Equals(b.Reference,StringComparison.OrdinalIgnoreCase));
    private static bool NamedMatch(PreparedSegment a, PreparedSegment b) =>
        a.Reference.Length > 0 && a.Reference.Equals(b.Reference,StringComparison.OrdinalIgnoreCase) || a.Name.Length > 0 && a.Name.Equals(b.Name,StringComparison.OrdinalIgnoreCase);
    private static float2 Heading(PreparedSegment s) => math.normalizesafe(s.End.xz-s.Start.xz);
    private static float Length(PreparedSegment s) => math.distance(s.Start.xz,s.End.xz);

    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source, ImportLog log, CancellationToken token, bool rail)
    {
        var roads = source.Where(s=>s.Kind==NetworkKind.Road).ToList();
        var tracks = source.Where(s=>s.Kind==NetworkKind.Rail).ToList();
        if (roads.Count > 0) { roads = RemoveCoincident(roads,log,token); roads = AssociateCorridors(roads,log,token); roads = ProtectCarriageways(roads,log,token); roads = ConstrainWidths(roads,token); roads = ReconnectBridges(roads,log,token); }
        if (rail && tracks.Count > 0) tracks = SimplifyYards(tracks,log,token);
        return roads.Concat(rail ? tracks : Enumerable.Empty<PreparedSegment>()).ToArray();
    }

    private static List<PreparedSegment> RemoveCoincident(List<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var buckets = new Dictionary<(int,int,int,int),List<PreparedSegment>>();
        var kept = new List<PreparedSegment>(); var remap = new Dictionary<(int,bool,bool,long),long>();
        foreach (var s in source.OrderBy(s=>s.WayId).ThenBy(s=>s.StartNodeId))
        {
            token.ThrowIfCancellationRequested();
            var a=s.Start.xz;var b=s.End.xz;
            if(!s.OneWay && (a.x>b.x || a.x==b.x && a.y>b.y)){var swap=a;a=b;b=swap;}
            var key=((int)Math.Round(a.x*10),(int)Math.Round(a.y*10),(int)Math.Round(b.x*10),(int)Math.Round(b.y*10));
            if(!buckets.TryGetValue(key,out var candidates)) buckets[key]=candidates=new List<PreparedSegment>();
            PreparedSegment? duplicate=null;
            foreach(var other in candidates)
            {
                if(!SameLevel(s,other)||s.Classification!=other.Classification||s.Lanes!=other.Lanes||s.OneWay!=other.OneWay||ConflictingNames(s,other))continue;
                var reversed=math.distance(s.Start.xz,other.End.xz)<.05f;
                if(Enumerable.Range(0,33).All(i=>math.distance(NetworkNormalizer.Evaluate(s,i/32f).xz,NetworkNormalizer.Evaluate(other,(reversed?32-i:i)/32f).xz)<.05f)) {duplicate=other;break;}
            }
            if(duplicate==null){kept.Add(s);candidates.Add(s);continue;}
            var reverse=math.distance(s.Start.xz,duplicate.End.xz)<.05f;
            remap[(s.Layer,s.Bridge,s.Tunnel,s.StartNodeId)]=reverse?duplicate.EndNodeId:duplicate.StartNodeId;
            remap[(s.Layer,s.Bridge,s.Tunnel,s.EndNodeId)]=reverse?duplicate.StartNodeId:duplicate.EndNodeId;
            log.Count("Coincident road representations removed");
        }
        long Map(PreparedSegment s,long node){for(var i=0;i<16&&remap.TryGetValue((s.Layer,s.Bridge,s.Tunnel,node),out var next)&&next!=node;i++)node=next;return node;}
        return kept.Select(s=>s with{StartNodeId=Map(s,s.StartNodeId),EndNodeId=Map(s,s.EndNodeId)}).ToList();
    }

    private static List<PreparedSegment> AssociateCorridors(List<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var grid=Index(source,128); var evidence=new Dictionary<(long,long),(float overlap,int count,bool named)>();
        foreach(var a in source.Where(s=>s.OneWay&&!s.ProtectedLink))
        {
            token.ThrowIfCancellationRequested();
            foreach(var j in Neighbors(grid,(a.Start.xz+a.End.xz)*.5f,128))
            {
                var b=source[j];if(a.WayId>=b.WayId||!b.OneWay||b.ProtectedLink||a.Classification!=b.Classification||!SameLevel(a,b)||ConflictingNames(a,b))continue;
                var ha=Heading(a);var hb=Heading(b);if(math.dot(ha,hb)>-.97f)continue;
                var length=Length(a);if(length<10||Length(b)<10)continue;
                var relative=b.Start.xz-a.Start.xz;var separation=Math.Abs(ha.x*relative.y-ha.y*relative.x);
                if(separation<2||separation>40)continue;
                var endSeparation=Math.Abs(ha.x*(b.End.z-a.Start.z)-ha.y*(b.End.x-a.Start.x));if(Math.Abs(endSeparation-separation)>3)continue;
                var p=math.dot(relative,ha);var q=math.dot(b.End.xz-a.Start.xz,ha);
                var overlap=Math.Min(length,Math.Max(p,q))-Math.Max(0,Math.Min(p,q));
                if(overlap<Math.Min(length,Length(b))*.7f)continue;
                var key=(a.WayId,b.WayId);evidence.TryGetValue(key,out var old);evidence[key]=(old.overlap+overlap,old.count+1,old.named||NamedMatch(a,b));
            }
        }
        var parent=new Dictionary<long,long>();
        long Root(long id){if(!parent.TryGetValue(id,out var p))return id;while(parent.TryGetValue(p,out var next)&&next!=p)p=next;return p;}
        foreach(var pair in evidence.OrderBy(p=>p.Key.Item1).ThenBy(p=>p.Key.Item2))
        {
            if(!(pair.Value.named&&pair.Value.overlap>=30 || pair.Value.count>=3&&pair.Value.overlap>=150)) {log.Count("Ambiguous parallel road pairs preserved");continue;}
            var a=Root(pair.Key.Item1);var b=Root(pair.Key.Item2);if(a==b)continue;parent[Math.Max(a,b)]=Math.Min(a,b);parent[Math.Min(a,b)]=Math.Min(a,b);log.Count("Divided corridors detected");
        }
        var paired = source.Select(s=>parent.ContainsKey(s.WayId)?s with{CorridorId=Root(s.WayId)}:s).ToList();
        return paired;
    }

    private static List<PreparedSegment> ConstrainWidths(List<PreparedSegment> paired, CancellationToken token)
    {
        var pairedGrid=Index(paired,128);
        for(var i=0;i<paired.Count;i++)
        {
            token.ThrowIfCancellationRequested();var a=paired[i];if(a.CorridorId==0)continue;var limit=float.PositiveInfinity;
            foreach(var j in Neighbors(pairedGrid,(a.Start.xz+a.End.xz)*.5f,128))
            {
                var b=paired[j];if(a.WayId==b.WayId||a.CorridorId!=b.CorridorId||!SameLevel(a,b)||math.dot(Heading(a),Heading(b))>-.9f)continue;
                // A genuine shared source junction is handled by the native junction
                // builder; its zero separation is not a carriageway width failure.
                if(a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId)continue;
                for(var sample=0;sample<=8;sample++)
                {
                    var point=NetworkNormalizer.Evaluate(a,sample/8f);var closest=float.PositiveInfinity;
                    for(var step=1;step<=16;step++)closest=Math.Min(closest,NetworkNormalizer.DistanceToSegment(point,NetworkNormalizer.Evaluate(b,(step-1)/16f),NetworkNormalizer.Evaluate(b,step/16f)));
                    if(closest<40)limit=Math.Min(limit,Math.Max(0,closest-1));
                }
            }
            paired[i]=a with{MaximumNativeWidth=limit};
        }
        return paired;
    }

    private static List<PreparedSegment> ProtectCarriageways(List<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        HashSet<int> Crossed(List<PreparedSegment> items)
        {
            var crossed=new HashSet<int>();var grid=Index(items,128);
            for(var i=0;i<items.Count;i++)
            {
                token.ThrowIfCancellationRequested();var a=items[i];if(a.CorridorId==0)continue;
                foreach(var j in Neighbors(grid,(a.Start.xz+a.End.xz)*.5f,128))
                {
                    var b=items[j];if(i>=j||a.WayId==b.WayId||a.CorridorId!=b.CorridorId||!SameLevel(a,b)||a.StartNodeId==b.StartNodeId||a.StartNodeId==b.EndNodeId||a.EndNodeId==b.StartNodeId||a.EndNodeId==b.EndNodeId)continue;
                    bool collision=false;
                    for(var p=1;p<=16&&!collision;p++)for(var q=1;q<=16;q++)
                        if(Intersects(NetworkNormalizer.Evaluate(a,(p-1)/16f).xz,NetworkNormalizer.Evaluate(a,p/16f).xz,NetworkNormalizer.Evaluate(b,(q-1)/16f).xz,NetworkNormalizer.Evaluate(b,q/16f).xz)){collision=true;break;}
                    if(collision){crossed.Add(i);crossed.Add(j);}
                }
            }
            return crossed;
        }
        var collisions=Crossed(source);if(collisions.Count==0)return source;
        var restored=new List<PreparedSegment>();
        for(var index=0;index<source.Count;index++)
        {
            var s=source[index];if(!collisions.Contains(index)||s.SourceGeometry.Count<2){restored.Add(s);continue;}
            log.Count("Carriageway fits reverted to source topology");
            for(var p=1;p<s.SourceGeometry.Count;p++)
            {
                long InternalId(float3 point){unchecked{return -1-(long)(((ulong)s.WayId*104729UL ^ math.hash(point.xz))%(ulong)long.MaxValue);}}
                var start=s.SourceGeometry[p-1];var end=s.SourceGeometry[p];
                restored.Add(s with{Start=start,End=end,StartNodeId=p==1?s.StartNodeId:InternalId(start),EndNodeId=p==s.SourceGeometry.Count-1?s.EndNodeId:InternalId(end),Control1=null,Control2=null,SourceGeometry=new[]{start,end},HorizontalParent=null});
            }
        }
        var unsafeIndices=Crossed(restored);
        if(unsafeIndices.Count>0)log.Warn($"Horizontal network validation: {unsafeIndices.Count} carriageway sections still cross without shared source topology after restoring source geometry; omitted before placement.");
        log.Count("Unsafe carriageway sections omitted",unsafeIndices.Count);
        if(unsafeIndices.Count>0&&unsafeIndices.Count>restored.Count/2)throw new InvalidOperationException("Major horizontal network failure: conflicting carriageways remain after source-geometry fallback. Current map unchanged.");
        return restored.Where((s,i)=>!unsafeIndices.Contains(i)).ToList();
    }
    private static bool Intersects(float2 a,float2 b,float2 c,float2 d)
    {
        float Cross(float2 x,float2 y)=>x.x*y.y-x.y*y.x;
        var r=b-a;var s=d-c;var divisor=Cross(r,s);if(Math.Abs(divisor)<.00001f)return false;
        var t=Cross(c-a,s)/divisor;var u=Cross(c-a,r)/divisor;
        return t>=-.0001f&&t<=1.0001f&&u>=-.0001f&&u<=1.0001f;
    }

    private static List<PreparedSegment> ReconnectBridges(List<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var degree=new Dictionary<long,int>();foreach(var s in source)foreach(var n in new[]{s.StartNodeId,s.EndNodeId}){degree.TryGetValue(n,out var d);degree[n]=d+1;}
        var grid=Index(source,128);var result=new List<PreparedSegment>(source);var used=new HashSet<long>();long synthetic=-9_000_000_000_000_000_000;
        foreach(var bridge in source.Where(s=>s.Bridge&&!s.Tunnel)) foreach(var atStart in new[]{true,false})
        {
            token.ThrowIfCancellationRequested();var node=atStart?bridge.StartNodeId:bridge.EndNodeId;if(degree[node]>1||used.Contains(node))continue;
            var position=atStart?bridge.Start:bridge.End;var outward=Heading(bridge)*(atStart?-1:1);
            var candidates=new List<(PreparedSegment road,bool start,float distance)>();
            foreach(var i in Neighbors(grid,position.xz,128))
            {
                var road=source[i];if(road.WayId==bridge.WayId&&road.Bridge || road.Tunnel||road.Bridge||road.Classification!=bridge.Classification||road.OneWay!=bridge.OneWay||ConflictingNames(bridge,road))continue;
                if(!(road.WayId==bridge.WayId||NamedMatch(bridge,road)||road.CorridorId!=0&&road.CorridorId==bridge.CorridorId))continue;
                foreach(var roadStart in new[]{true,false})
                {
                    var roadNode=roadStart?road.StartNodeId:road.EndNodeId;if(degree[roadNode]!=1||used.Contains(roadNode))continue;
                    if(bridge.OneWay && roadStart==atStart)continue; // incoming end -> bridge start, bridge end -> outgoing start
                    var endpoint=roadStart?road.Start:road.End;var delta=endpoint.xz-position.xz;var distance=math.length(delta);
                    if(distance<.05f||distance>30||math.dot(math.normalizesafe(delta),outward)<.97f)continue;
                    var heading=Heading(road)*(roadStart?1:-1);if(math.dot(heading,outward)<.97f)continue;
                    if(Math.Abs(outward.x*delta.y-outward.y*delta.x)>1)continue; // never hop sideways onto the paired deck
                    candidates.Add((road,roadStart,distance));
                }
            }
            candidates=candidates.OrderBy(c=>c.distance).ThenBy(c=>c.road.WayId).ToList();
            if(candidates.Count==0 || candidates.Count>1&&candidates[1].distance-candidates[0].distance<3) {log.Count("Bridge approaches unresolved");continue;}
            var best=candidates[0];var roadPoint=best.start?best.road.Start:best.road.End;var roadId=best.start?best.road.StartNodeId:best.road.EndNodeId;
            var connector=best.road with{WayId=synthetic++,Start=atStart?roadPoint:position,End=atStart?position:roadPoint,StartNodeId=atStart?roadId:node,EndNodeId=atStart?node:roadId,Control1=null,Control2=null,SourceGeometry=new[]{atStart?roadPoint:position,atStart?position:roadPoint},CorridorId=bridge.CorridorId};
            result.Add(connector);used.Add(node);used.Add(roadId);log.Count("Bridge approaches resolved");
        }
        return result;
    }

    private static List<PreparedSegment> SimplifyYards(List<PreparedSegment> source, ImportLog log, CancellationToken token)
    {
        var ways=source.GroupBy(s=>s.WayId).OrderBy(g=>g.Key).Select(g=>g.ToArray()).ToArray();
        var eligible=ways.Where(g=>g.All(s=>s.RailService=="yard"||s.RailService=="siding"||s.RailService=="spur")&&!g.Any(s=>s.Bridge||s.Tunnel)).ToArray();
        var removed=new HashSet<long>();var yardCount=0;var retained=0;var sourceCount=0;
        float3 Mid(PreparedSegment[] group)=>NetworkNormalizer.Evaluate(group[group.Length/2],.5f);
        bool Parallel(PreparedSegment[] a,PreparedSegment[] b)
        {
            if(a[0].WayId==b[0].WayId||!SameLevel(a[0],b[0])||math.abs(math.dot(Heading(a[0]),Heading(b[0])))<.98f)return false;
            var heading=Heading(a[0]);var start=a[0].Start.xz;var length=math.distance(start,a[a.Length-1].End.xz);
            var p=math.dot(b[0].Start.xz-start,heading);var q=math.dot(b[b.Length-1].End.xz-start,heading);
            return length>20 && Math.Min(length,Math.Max(p,q))-Math.Max(0,Math.Min(p,q))>length*.7f && math.distance(Mid(a).xz,Mid(b).xz)<6;
        }
        var dense=new HashSet<long>(eligible.Where(a=>{var neighbors=eligible.Where(b=>Parallel(a,b)).ToArray();return neighbors.Length>=3 && (a.Any(s=>s.RailService=="yard")||neighbors.Any(b=>b.Any(s=>s.RailService=="yard")));}).Select(g=>g[0].WayId));
        // Only redundant service tracks with an existing alternate topology path may be removed.
        // Mainlines, ordinary double track, station throats and disconnected terminal spurs are untouched.
        foreach(var group in eligible.Where(g=>dense.Contains(g[0].WayId)).OrderBy(g=>g[0].WayId))
        {
            token.ThrowIfCancellationRequested();var representative=group[0];
            var close=eligible.Where(g=>!removed.Contains(g[0].WayId)&&Parallel(group,g)).ToArray();
            if(close.Length==0)continue;
            var axis=Heading(representative);var middle=Mid(group).xz;
            var offsets=close.Select(g=>{var p=Mid(g).xz-middle;return axis.x*p.y-axis.y*p.x;}).ToArray();
            // Preserve both outside tracks/yard envelope, regardless of source count.
            if(!offsets.Any(v=>v<-.1f)||!offsets.Any(v=>v>.1f)){retained++;continue;}
            sourceCount++;var start=group[0].StartNodeId;var end=group[group.Length-1].EndNodeId;
            if(start==end||!AlternatePath(start,end,representative.WayId)) {retained++;continue;}
            removed.Add(representative.WayId);yardCount++;
        }
        if(sourceCount>0){log.Count("Rail yard service tracks examined",sourceCount);log.Count("Redundant yard tracks removed",yardCount);log.Count("Yard tracks retained",retained);}
        return source.Where(s=>!removed.Contains(s.WayId)).ToList();
        bool AlternatePath(long start,long end,long excluded)
        {
            var adjacency=new Dictionary<long,List<long>>();
            foreach(var s in source.Where(s=>s.WayId!=excluded&&!removed.Contains(s.WayId)))
            { if(!adjacency.TryGetValue(s.StartNodeId,out var a))adjacency[s.StartNodeId]=a=new List<long>();a.Add(s.EndNodeId);if(!adjacency.TryGetValue(s.EndNodeId,out var b))adjacency[s.EndNodeId]=b=new List<long>();b.Add(s.StartNodeId); }
            var seen=new HashSet<long>{start};var queue=new List<long>{start};var cursor=0;
            while(cursor<queue.Count&&seen.Count<=2000){token.ThrowIfCancellationRequested();var n=queue[cursor++];if(n==end)return true;if(adjacency.TryGetValue(n,out var next))foreach(var v in next)if(seen.Add(v))queue.Add(v);}
            return false;
        }
    }
    private static Dictionary<(int,int),List<int>> Index(List<PreparedSegment> source,float cell)
    {
        var index=new Dictionary<(int,int),List<int>>();for(var i=0;i<source.Count;i++){var s=source[i];var p=(s.Start.xz+s.End.xz)*.5f;var key=((int)Math.Floor(p.x/cell),(int)Math.Floor(p.y/cell));if(!index.TryGetValue(key,out var list))index[key]=list=new List<int>();list.Add(i);}return index;
    }
    private static IEnumerable<int> Neighbors(Dictionary<(int,int),List<int>> grid,float2 p,float cell)
    { var x=(int)Math.Floor(p.x/cell);var y=(int)Math.Floor(p.y/cell);for(var dy=-2;dy<=2;dy++)for(var dx=-2;dx<=2;dx++)if(grid.TryGetValue((x+dx,y+dy),out var found))foreach(var i in found)yield return i; }
}
