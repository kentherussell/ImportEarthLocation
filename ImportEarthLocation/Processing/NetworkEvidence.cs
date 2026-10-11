using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>One normalized evidence snapshot, independent of mutable designed geometry.</summary>
public static class NetworkEvidence
{
    private static readonly HashSet<string> SemanticKeys=new(StringComparer.OrdinalIgnoreCase)
    {"highway","railway","oneway","bridge","tunnel","covered","location","junction","cutting","retaining_wall","motorroad","expressway","access","service","noexit","leaf_type","leaf_cycle","type","restriction"};

    public static OsmDataset Normalize(OsmDataset source,ImportRequest request,ImportLog log,CancellationToken token)
    {
        IReadOnlyDictionary<string,string> Tags(IReadOnlyDictionary<string,string> raw)=>raw.GroupBy(p=>p.Key.Trim(),StringComparer.OrdinalIgnoreCase).ToDictionary(g=>g.Key,g=>SemanticKeys.Contains(g.Key)?g.Last().Value.Trim().ToLowerInvariant():g.Last().Value.Trim(),StringComparer.OrdinalIgnoreCase);
        var ways=new Dictionary<long,OsmWay>();
        foreach(var w in source.Ways.Values){token.ThrowIfCancellationRequested();ways[w.Id]=w with{Tags=Tags(w.Tags)};}
        var selected=new HashSet<long>(ways.Values.Where(w=>w.Tags.TryGetValue("highway",out var h)&&NetworkSelection.Includes(request.RoadLevel,h)||request.ImportRail&&w.Tags.TryGetValue("railway",out var r)&&r=="rail").Select(w=>w.Id));
        var selectedNodes=new HashSet<long>(selected.SelectMany(id=>ways[id].NodeIds));
        var support=new HashSet<long>();
        var relations=source.Relations.Select(r=>r with{Tags=Tags(r.Tags),OrderedMembers=r.OrderedMembers.Select(m=>m with{Type=m.Type.Trim().ToLowerInvariant(),Role=m.Role.Trim().ToLowerInvariant()}).ToArray()}).ToArray();
        foreach(var relation in relations)
        {
            var members=relation.OrderedMembers;
            if(!members.Any(m=>m.Type=="way"&&m.Role=="from"&&selected.Contains(m.Id))||!members.Any(m=>m.Type=="way"&&m.Role=="to"&&selected.Contains(m.Id)))continue;
            foreach(var via in members.Where(m=>m.Type=="way"&&m.Role=="via"))if(ways.ContainsKey(via.Id))support.Add(via.Id);
        }
        foreach(var w in ways.Values.Where(w=>!selected.Contains(w.Id)&&w.Tags.ContainsKey("highway")))
        {
            token.ThrowIfCancellationRequested();
            var joins=w.NodeIds.Count(selectedNodes.Contains);
            var ring=w.Tags.TryGetValue("junction",out var j)&&j=="roundabout";
            if(joins==0||!ring&&joins<2)continue;
            var length=0f;
            for(var i=1;i<w.NodeIds.Length;i++)if(source.Nodes.TryGetValue(w.NodeIds[i-1],out var a)&&source.Nodes.TryGetValue(w.NodeIds[i],out var b))
                length+=math.distance(GeoMath.ToLocalMeters(a.Latitude,a.Longitude,request.Latitude,request.Longitude).xz,GeoMath.ToLocalMeters(b.Latitude,b.Longitude,request.Latitude,request.Longitude).xz);
            if(ring||length<=500)support.Add(w.Id);
        }
        var required=selected.Concat(support).Distinct().ToArray();
        // Complete split roundabout components, not just the arc touching a selected
        // road. The downloaded query is bounded; this walks only its ring subgraph.
        var ringNodes=new HashSet<long>(selected.Concat(support).Where(id=>ways[id].Tags.TryGetValue("junction",out var j)&&j=="roundabout").SelectMany(id=>ways[id].NodeIds));
        var pending=ways.Values.Where(w=>!selected.Contains(w.Id)&&!support.Contains(w.Id)&&w.Tags.TryGetValue("junction",out var j)&&j=="roundabout").ToList();
        bool changed;
        do {changed=false;for(var i=pending.Count-1;i>=0;i--){token.ThrowIfCancellationRequested();var w=pending[i];if(!w.NodeIds.Any(ringNodes.Contains))continue;support.Add(w.Id);ringNodes.UnionWith(w.NodeIds);pending.RemoveAt(i);changed=true;}} while(changed);
        required=selected.Concat(support).Distinct().ToArray();
        var missing=required.SelectMany(id=>ways[id].NodeIds).Where(id=>!source.Nodes.ContainsKey(id)).Distinct().Take(20).ToArray();
        if(missing.Length>0)throw new PreparationDataException(PreparationDataIssue.NetworkIncomplete,$"Downloaded network still has missing required node references after bounded dependency recovery ({string.Join(",",missing)}).");
        log.Count("Junction support ways admitted",support.Count);
        log.Diagnostic($"NETWORK EVIDENCE selectedWays={selected.Count}; supportWays={support.Count}; taggedNodes={source.NodeTags.Count}; restrictions={source.Relations.Count(r=>r.Tags.TryGetValue("type",out var t)&&t=="restriction")}; connectivity={source.Relations.Count(r=>r.Tags.TryGetValue("type",out var t)&&t=="connectivity")}; dependencyClosure=true; interpretationVersion=3; newEndpoints=0");
        foreach(var r in source.Relations.Where(r=>r.Tags.TryGetValue("type",out var t)&&(t=="restriction"||t=="connectivity")).Take(40))
            log.Diagnostic($"MOVEMENT EVIDENCE relation={r.Id}; members={string.Join("|",r.OrderedMembers.Select(m=>$"{m.Type}:{m.Id}:{m.Role}"))}; policy={string.Join("|",r.Tags.Where(p=>p.Key.StartsWith("restriction")||p.Key=="connectivity"||p.Key=="except").Select(p=>p.Key+"="+p.Value))}; nativeEnforcementRequiresReadback=true");
        return new OsmDataset{Nodes=source.Nodes,NodeTags=source.NodeTags.ToDictionary(p=>p.Key,p=>Tags(p.Value)),Ways=ways,Relations=relations,SupportWayIds=support,RawWayTags=source.Ways.ToDictionary(p=>p.Key,p=>p.Value.Tags),MalformedFeaturesSkipped=source.MalformedFeaturesSkipped};
    }

    public static bool NodeIs(OsmDataset data,long id,string key,string value)=>data.NodeTags.TryGetValue(id,out var tags)&&tags.TryGetValue(key,out var actual)&&actual==value;
    public static float? Measure(IReadOnlyDictionary<string,string> tags,string key,bool speed=false)
    {
        if(!tags.TryGetValue(key,out var text))return null;
        text=text.Trim().ToLowerInvariant();var factor=1f;
        if(speed&&text.EndsWith("mph")){factor=1.609344f;text=text.Substring(0,text.Length-3).Trim();}
        else if(!speed&&(text.EndsWith("ft")||text.EndsWith("feet"))){factor=.3048f;text=text.Substring(0,text.Length-(text.EndsWith("ft")?2:4)).Trim();}
        else text=text.Replace(speed?"km/h":"m","").Trim();
        return float.TryParse(text,NumberStyles.Float,CultureInfo.InvariantCulture,out var number)&&TerrainConditioner.Finite(number)&&number>0&&number*factor<=(speed?250:100)?number*factor:null;
    }
}
