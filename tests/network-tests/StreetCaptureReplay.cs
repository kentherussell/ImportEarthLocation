using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    // Recover the complete captured source, rather than running consolidation
    // a second time on an already consolidated design boundary.
    public static void Streets(string folder,string output,bool design)
    {
        var c=Load(folder);var request=c.Input["request"]!.ToObject<ImportRequest>()!;
        Dictionary<long,T> Read<T>(string name)=>CorrectiveCapturedReplay.ReadManifest<KeyValuePair<long,T>>(folder,"offline-osm-"+name+"-chunks.json").ToDictionary(p=>p.Key,p=>p.Value);
        var metadata=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-osm-metadata.json")));
        var data=new OsmDataset{Nodes=Read<OsmNode>("nodes"),Ways=Read<OsmWay>("ways"),
            NodeTags=Read<IReadOnlyDictionary<string,string>>("node-tags"),RawWayTags=Read<IReadOnlyDictionary<string,string>>("raw-way-tags"),
            Relations=CorrectiveCapturedReplay.ReadManifest<OsmRelation>(folder,"offline-osm-relations-chunks.json"),
            SupportWayIds=metadata["SupportWayIds"]!.ToObject<long[]>()!,MalformedFeaturesSkipped=(int)metadata["MalformedFeaturesSkipped"]!};
        var log=new ImportLog{ScalarDiagnostics=true};
        IReadOnlyList<PreparedSegment> source=NetworkProcessor.BuildSegments(NetworkEvidence.Normalize(data,request,log,default),request,log);
        source=CoordinatedNetworkDesign.TunnelIntervals(source,log,default);
        source=NetworkStubs.Prune(source,request,log);source=NetworkLaneRuns.Smooth(source,log);
        source=NetworkOverlapConsolidator.Prepare(source,log,default);
        var before=source;source=MedianLinks.Prepare(source,log,default);
        source=AvenueConsolidator.PrepareTrunks(source,log,default);
        foreach(var line in log.Lines.Where(l=>l.Contains("TRUNK AVENUE",StringComparison.Ordinal)))Console.WriteLine(line);
        foreach(var metric in new[]{"Divided trunk pairs consolidated","Divided trunk stretches kept without intersections","Median links folded into merged trunks",
            "Divided trunk carriageway sections reshaped","Divided trunk transitions limited by nearby junctions or structures"})Console.WriteLine($"{metric}: {log.Metric(metric)}");
        source=AvenueConsolidator.Prepare(source,log,default);
        source=NetworkLaneRuns.NormalizeStreets(source,log);
        source=StreetJunctionClusters.Prepare(source,log,default);
        foreach(var line in log.Lines.Where(l=>l.Contains("STREET JUNCTION CLUSTERS",StringComparison.Ordinal)||l.Contains("STREET SPLITS",StringComparison.Ordinal)||l.Contains("MEDIAN LINKS",StringComparison.Ordinal)||l.Contains("TURN SLIPS",StringComparison.Ordinal)))Console.WriteLine(line);
        var obligations=new ImportLog{CaptureRecords=true};
        SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(before),source,obligations,default);
        Directory.CreateDirectory(output);
        File.WriteAllText(Path.Combine(output,"avenues.json"),JsonConvert.SerializeObject(new{
            capture=Path.GetFileName(folder),completeCapturedOsm=true,beforeSections=before.Count,afterSections=source.Count,
            pairs=log.Metric("Opposing road pairs consolidated into avenues"),
            streetWidthSteps=log.Metric("Street intersection width steps normalized"),
            intersectionBoxes=log.Metric("Divided street intersection boxes consolidated"),
            roundaboutWayIds=before.Where(s=>s.Roundabout).Select(s=>s.WayId).Distinct().ToArray(),
            highwayIntervals=log.Metric("Highway coincident intervals consolidated"),
            highwayIntervalDiagnostics=log.Lines.Where(l=>l.Contains("HIGHWAY INTERVAL",StringComparison.Ordinal)).ToArray(),
            missingPorts=obligations.Metric("Missing mandatory selected-source ports"),unresolvedRoutes=obligations.Metric("Unresolved selected-source reachability obligations"),
            failures=obligations.Records.Values.SelectMany(r=>r).Select(JObject.FromObject).Where(r=>(bool?)r["reachable"]==false).ToArray(),
            diagnostics=log.Lines.Where(l=>l.Contains("AVENUE ",StringComparison.Ordinal)).ToArray(),
            retained=source.Where(s=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.Classification.StartsWith("motorway")&&!s.Classification.StartsWith("trunk")&&!s.Roundabout)
                .GroupBy(s=>new{s.Name,s.Reference,s.Classification}).Select(g=>new{g.Key,sections=g.Count(),ways=g.Select(s=>s.WayId).Distinct(),length=g.Sum(s=>math.distance(s.Start.xz,s.End.xz))})
                .OrderByDescending(g=>g.length).ToArray()},Formatting.Indented));
        using(var file=File.Create(Path.Combine(output,"street-sections.ndjson.gz")))
        using(var zip=new System.IO.Compression.GZipStream(file,System.IO.Compression.CompressionLevel.Optimal))
        using(var writer=new StreamWriter(zip))foreach(var s in source)writer.WriteLine(JsonConvert.SerializeObject(new{
            s.WayId,s.StartNodeId,s.EndNodeId,s.Kind,s.Name,s.Reference,s.Classification,s.OneWay,s.Lanes,s.Bridge,s.Tunnel,s.Layer,s.Roundabout,s.ConsolidatedAvenue,
            start=new[]{s.Start.x,s.Start.y,s.Start.z},end=new[]{s.End.x,s.End.y,s.End.z},s.SourceWayIds,
            geometry=s.SourceGeometry.SelectMany(p=>new[]{p.x,p.z}).ToArray()}));
        Console.WriteLine($"Street replay: {before.Count} -> {source.Count} sections; {log.Metric("Opposing road pairs consolidated into avenues")} pairs; missing ports={obligations.Metric("Missing mandatory selected-source ports")}; unresolved={obligations.Metric("Unresolved selected-source reachability obligations")}");
        if(!design)return;
        void Audit(string stage)
        {
            var audit=new ImportLog();SourceTopologyObligations.Validate(SourceTopologyObligations.Capture(source),source,audit,default);
            Console.WriteLine($"Source self-audit {stage}: missing={audit.Metric("Missing mandatory selected-source ports")}; routes={audit.Metric("Unresolved selected-source reachability obligations")}");
        }
        Audit("avenues");
        source=RoundaboutPlanner.Prepare(source,log,default);
        Audit("roundabouts");
        if(request.ImportRail)source=RailCorridorConsolidator.Prepare(source,log,default);
        source=StarterNetworkPolicy.Rail(source,log,default);
        source=NetworkNormalizer.Normalize(source,log,default,c.Width);
        Audit("normalization");
        source=NetworkHardener.Prepare(source,log,default,request.ImportRail);
        Audit("hardening");
        source=OsmRecovery.RepairGeometry(source,c.Width,log,default);
        var palette=JObject.Parse(File.ReadAllText(CapturedRunFiles.Resolve(folder,"offline-palette-input.json")));
        var widths=palette["widths"]!.ToObject<Dictionary<string,float>>()!;
        var offsets=palette["laneOffsets"]!.ToObject<Dictionary<string,float[]>>()!;
        source=source.Select(s=>s with{NativeWidth=widths.GetValueOrDefault(NativeAssetKeys.Sized(s)),
            NativeLaneOffsets=s.Kind==NetworkKind.Rail?Array.Empty<float>():offsets.GetValueOrDefault(NativeAssetKeys.Sized(s),Array.Empty<float>())}).ToArray();
        source=NativeJunctionSimplifier.Prepare(source,log,default);
        Audit("native-junctions");
        source=NetworkProcessor.AlignToTerrain(source,c.Terrain,c.Scale,c.Width,default);
        source=CoordinatedNetworkDesign.UnmappedCrossingBridges(source,log,default);
        source=CoordinatedNetworkDesign.StructureIntervals(source,c.Terrain,c.Scale,c.Width,log,default,c.Water,tunnelIntervalsResolved:true);
        Audit("structure-intervals");
        source=source.Select(s=>s with{NativeAssetKey=NativeAssetKeys.Sized(s),
            NativeWidth=widths.GetValueOrDefault(NativeAssetKeys.Sized(s)),
            NativeLaneOffsets=s.Kind==NetworkKind.Rail?Array.Empty<float>():offsets.GetValueOrDefault(NativeAssetKeys.Sized(s),Array.Empty<float>())}).ToArray();
        var missingAssets=source.Select(s=>s.NativeAssetKey).Distinct().Except(c.Contracts.Select(a=>a.PaletteKey)).ToArray();
        if(missingAssets.Length>0)Console.WriteLine("Captured native envelope contracts unavailable for: "+string.Join(",",missingAssets));
        // Water/terrain and native contracts stay frozen for the paired experiment.
        // This measures source-road changes, not a fresh end-to-end game import.
        Measure(c with{Source=source.ToArray()},output,"captured-source-visual-compiler",(capture,l)=>
            VisualNetworkCompiler.Prepare(capture.Source,capture.Terrain,capture.Scale,capture.Width,l,default,capture.Water));
    }
}
