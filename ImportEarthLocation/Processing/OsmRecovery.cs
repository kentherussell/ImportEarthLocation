using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// OSM enrichment is best effort. Cancellation/resource failures are not data errors.
public static class OsmRecovery
{
    public static bool Recoverable(Exception error) => error is InvalidOperationException ||
        error is InvalidDataException || error is ArgumentException || error is FormatException ||
        error is OverflowException || error is KeyNotFoundException || error is IndexOutOfRangeException ||
        error is System.Xml.XmlException || error is System.Net.Http.HttpRequestException || error is TimeoutException;

    public static T Run<T>(string stage, Func<T> work, T fallback, ImportLog log, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        try { return work(); }
        catch (Exception error) when (Recoverable(error))
        {
            token.ThrowIfCancellationRequested();
            log.Count("OSM recovery stages");
            log.Warn($"OSM {stage}: {error.Message} Using retained data/fallback; preparation continues. See diagnostics for limitations.");
            log.Diagnostic($"OSM RECOVERY stage={stage}; exception={error.GetType().Name}; detail={error}; generationContinues=true; resultNotFullyVerified=true");
            return fallback;
        }
    }

    public static OsmDataset Parse(IReadOnlyList<byte[]> documents, string cacheRoot, string category, ImportLog log, CancellationToken token)
    {
        if (documents.Count == 0) return new OsmDataset();
        try { return OsmParsedCache.Merge(documents, cacheRoot, log, token, wholeMap: documents.Count == 1); }
        catch (Exception error) when (Recoverable(error))
        {
            token.ThrowIfCancellationRequested();
            log.Warn($"OSM {category} batch could not be parsed ({error.Message}); recovering readable tiles individually.");
            var nodes = new Dictionary<long,OsmNode>(); var tags = new Dictionary<long,IReadOnlyDictionary<string,string>>();
            var ways = new Dictionary<long,OsmWay>(); var relations = new SortedDictionary<long,OsmRelation>(); var failed = 0; var malformed = 0;
            foreach (var document in documents)
            {
                token.ThrowIfCancellationRequested();
                try
                {
                    var data = OsmParser.Merge(new[]{document}, log, token, wholeMap:true);
                    foreach (var pair in data.Nodes) nodes[pair.Key] = pair.Value;
                    foreach (var pair in data.NodeTags) tags[pair.Key] = pair.Value;
                    foreach (var pair in data.Ways) ways[pair.Key] = pair.Value;
                    foreach (var relation in data.Relations) relations[relation.Id] = relation;
                    malformed += data.MalformedFeaturesSkipped;
                }
                catch (Exception tileError) when (Recoverable(tileError))
                { failed++; log.Warn($"OSM {category} tile unreadable: {tileError.Message}; other tiles retained."); }
            }
            log.Count("OSM unreadable tiles",failed);
            log.Diagnostic($"OSM RECOVERY stage=parse-{category}; readableTiles={documents.Count-failed}; failedTiles={failed}; ways={ways.Count}; coverageComplete=false; generationContinues=true");
            return new OsmDataset{Nodes=nodes,NodeTags=tags,Ways=ways,Relations=relations.Values.ToArray(),MalformedFeaturesSkipped=malformed};
        }
    }

    // Finite, bounded geometry and identical shared endpoints remain mandatory.
    // Repair local controls/junctions; only irrecoverable individual sections are omitted.
    public static IReadOnlyList<PreparedSegment> RepairGeometry(IReadOnlyList<PreparedSegment> source, double width, ImportLog log, CancellationToken token)
    {
        if (double.IsNaN(width) || double.IsInfinity(width) || width <= 0) throw new ArgumentException("Invalid map width.");
        var half=(float)(width/2); var nodes=new Dictionary<(NetworkKind,long),float3>(); var result=new List<PreparedSegment>(source.Count);
        var repaired=0;
        float3 Bound(float3 p) => new(math.clamp(p.x,-half,half),p.y,math.clamp(p.z,-half,half));
        // Recover missing anchors from the same known source node, even if its
        // healthy incident section occurs later. Never invent a nearest connection.
        foreach(var s in source)
        {
            token.ThrowIfCancellationRequested();
            foreach(var anchor in new[]{(s.StartNodeId,s.Start),(s.EndNodeId,s.End)})
                if(math.all(math.isfinite(anchor.Item2))&&!nodes.ContainsKey((s.Kind,anchor.Item1)))nodes[(s.Kind,anchor.Item1)]=Bound(anchor.Item2);
        }
        foreach (var segment in source)
        {
            token.ThrowIfCancellationRequested();
            float3 Recover(long id,float3 p,bool start)
            {
                if(math.all(math.isfinite(p)))return p;
                if(nodes.TryGetValue((segment.Kind,id),out var known))return known;
                if(segment.SourceGeometry.Count>=2)
                {var original=segment.SourceGeometry[start?0:segment.SourceGeometry.Count-1];if(math.all(math.isfinite(original)))return original;}
                return new float3(float.NaN);
            }
            float3 Node(long id,float3 p)
            {var key=(segment.Kind,id); if(nodes.TryGetValue(key,out var saved))return saved; return nodes[key]=Bound(p);}
            var anchorsRecovered=!math.all(math.isfinite(segment.Start))||!math.all(math.isfinite(segment.End));
            var recoveredStart=Recover(segment.StartNodeId,segment.Start,true);var recoveredEnd=Recover(segment.EndNodeId,segment.End,false);
            if(!math.all(math.isfinite(recoveredStart))||!math.all(math.isfinite(recoveredEnd)))
            {log.Count("Irrecoverable network sections omitted");log.Warn($"No finite source anchor for {segment.Kind} way {segment.WayId}; only this section omitted.");continue;}
            var a=Node(segment.StartNodeId,recoveredStart);var d=Node(segment.EndNodeId,recoveredEnd);
            if(math.distance(a,d)<.01f){log.Count("Degenerate network sections omitted");continue;}
            var changed=anchorsRecovered||math.distance(a,segment.Start)>.001f||math.distance(d,segment.End)>.001f;
            var c1=segment.Control1; var c2=segment.Control2;
            bool Valid(float3? p)=>!p.HasValue||math.all(math.isfinite(p.Value))&&Math.Abs(p.Value.x)<=half&&Math.Abs(p.Value.z)<=half;
            if(anchorsRecovered||!Valid(c1)||!Valid(c2)){c1=null;c2=null;changed=true;}
            else if(changed){if(c1.HasValue)c1=Bound(c1.Value+(a-segment.Start));if(c2.HasValue)c2=Bound(c2.Value+(d-segment.End));}
            var startElevation=segment.StartElevation.HasValue&&math.isfinite(segment.StartElevation.Value)&&math.isfinite(segment.Start.y)?segment.StartElevation+(a.y-segment.Start.y):null;
            var endElevation=segment.EndElevation.HasValue&&math.isfinite(segment.EndElevation.Value)&&math.isfinite(segment.End.y)?segment.EndElevation+(d.y-segment.End.y):null;
            var startOffset=math.all(math.isfinite(segment.StartCurveOffset))?segment.StartCurveOffset:default;
            var endOffset=math.all(math.isfinite(segment.EndCurveOffset))?segment.EndCurveOffset:default;
            changed|=!math.all(math.isfinite(segment.StartCurveOffset))||!math.all(math.isfinite(segment.EndCurveOffset));
            if(changed)repaired++;
            result.Add(segment with{Start=a,End=d,Control1=c1,Control2=c2,StartElevation=startElevation,EndElevation=endElevation,StartCurveOffset=startOffset,EndCurveOffset=endOffset,SourceGeometry=anchorsRecovered?new[]{a,d}:segment.SourceGeometry,HorizontalParent=changed?null:segment.HorizontalParent});
        }
        log.Count("OSM geometry sections repaired",repaired);
        if(repaired>0)log.Warn($"OSM geometry recovery: {repaired} sections repaired using known anchors/bounded controls; all {result.Count} sections retained.");
        return result;
    }
}
