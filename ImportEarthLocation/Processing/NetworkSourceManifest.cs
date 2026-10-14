using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using ImportEarthLocation.Geo;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

public static class NetworkSourceManifest
{
    // Capture BEFORE float conversion, consolidation or repair. Missing source
    // nodes end a run; never concatenate samples across an unavailable span.
    public static SourceManifest Build(OsmDataset raw, ImportRequest request, CancellationToken token)
    {
        var chains=new List<SourceChain>();
        foreach(var way in raw.Ways.Values.OrderBy(w=>w.Id))
        {
            token.ThrowIfCancellationRequested();
            NetworkKind kind;
            if(way.Tags.ContainsKey("highway"))kind=NetworkKind.Road;
            else if(way.Tags.TryGetValue("railway",out var rail)&&rail=="rail")kind=NetworkKind.Rail;
            else continue;
            var points=new List<SourcePoint>();var spans=new List<SourceSpanKey>();
            void Publish()
            {
                // A missing raw span can split one way into multiple chains.
                // Chain identity is unique in this immutable, way-ordered
                // manifest; original way identity remains in every span key.
                if(points.Count>1)chains.Add(new SourceChain(checked((long)chains.Count+1),points.ToArray(),spans.ToArray(),way.Tags));
                points.Clear();spans.Clear();
            }
            for(var i=0;i<way.NodeIds.Length;i++)
            {
                if((i&1023)==0)token.ThrowIfCancellationRequested();
                var id=way.NodeIds[i];
                if(!raw.Nodes.TryGetValue(id,out var node)){Publish();continue;}
                var p=GeoMath.ToLocalMetersDouble(node.Latitude,node.Longitude,request.Latitude,request.Longitude);
                if(!math.all(math.isfinite(p))){Publish();continue;}
                if(points.Count>0)spans.Add(new SourceSpanKey(way.Id,i-1,false));
                points.Add(new SourcePoint(new SourceNodeKey(kind,id),p));
            }
            Publish();
        }
        return new SourceManifest(chains.ToArray());
    }
}
