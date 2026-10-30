using System.IO.Compression;
using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Mathematics;

static partial class VisualMetricsReplay
{
    public static void RoadWater(string folder,string plot,string output)
    {
        var c=Load(folder);var wet=0;var under=0;var low=0;
        var worst=new Dictionary<long,(double Depth,double3 Point,float Level)>();
        using var reader=new StreamReader(new GZipStream(File.OpenRead(plot),CompressionMode.Decompress));
        while(reader.ReadLine() is {} line)
        {
            var row=JObject.Parse(line);if((string?)row["kind"]!="Road"||(bool)row["Tunnel"]!)continue;
            var p=row["p"]!.Values<double>().ToArray();var curve=new Cubic3d(new(p[0],p[1],p[2]),new(p[3],p[4],p[5]),new(p[6],p[7],p[8]),new(p[9],p[10],p[11]));
            var n=Math.Max(1,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve)/5));
            for(var k=0;k<=n;k++)
            {
                var t=k/(double)n;var q=NetworkDesignChecks.Evaluate(curve,t);
                var direction=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                var normal=new double2(-direction.y,direction.x)*(double)row["NativeWidth"]!*.5;
                for(var side=-1;side<=1;side++)
                {
                    var point=new float3((float)(q.x+normal.x*side),(float)q.y,(float)(q.z+normal.y*side));
                    var level=c.Water(point);if(!level.HasValue)continue;
                    wet++;var depth=level.Value-q.y;if(depth>-.25)low++;if(depth<=0)continue;under++;
                    var way=(long)row["WayId"]!;
                    if(!worst.TryGetValue(way,out var old)||depth>old.Depth)worst[way]=(depth,q,level.Value);
                }
            }
        }
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),historicalStages=true,nativeFlowVerified=false,
            wetSamples=wet,underwaterSamples=under,samplesUnder25cmFreeboard=low,
            worst=worst.OrderByDescending(p=>p.Value.Depth).Select(p=>new{way=p.Key,depth=p.Value.Depth,x=p.Value.Point.x,y=p.Value.Point.y,z=p.Value.Point.z,level=p.Value.Level}).ToArray()},Formatting.Indented));
        Console.WriteLine($"Road water: wet={wet}; underwater={under}; under25cm={low}");
    }
}
