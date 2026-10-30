using System;
using System.IO;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Data;

public sealed record NetworkPreferences(RoadImportLevel Roads, bool Rail, bool Forests = true, bool VisualDesign = false, bool DirectVegetation = false, bool Rocks = false, bool HdTerrain = false, bool Surfaces = true, bool AirRoutes = false, bool ShipRoutes = false)
{
    // Density sliders, 0–1 in tenths. A file without them takes 1 for an
    // enabled option and 0 for a disabled one.
    public float VegetationDensity { get; init; } = Forests ? 1 : 0;
    public float RockDensity { get; init; } = Rocks ? 1 : 0;
    public static float Step(float value) => (float)Math.Round(Math.Max(0, Math.Min(1, float.IsNaN(value) ? 1 : value)) * 10) / 10;
    public static NetworkPreferences Defaults => new(RoadImportLevel.None,true);
    private static string FilePath => Path.Combine(UnityEngine.Application.persistentDataPath,"ModsSettings",Mod.Id,"network-settings.json");
    public static NetworkPreferences Parse(string json)
    {
        try
        {
            var root=JObject.Parse(json);
            if(root["version"]!=null && (root["version"]!.Type!=JTokenType.Integer || (int)root["version"]!>2))return Defaults;
            var roads=root["version"]?.Value<int>()==2 && root["roadLevel"]?.Type==JTokenType.String?NetworkSelection.Parse((string?)root["roadLevel"]):RoadImportLevel.None;
            var rail=root["importRail"]?.Type==JTokenType.Boolean?(bool)root["importRail"]!:true;
            var forests=root["importForests"]?.Type==JTokenType.Boolean?(bool)root["importForests"]!:true;
            var visual=root["visualDesign"]?.Type==JTokenType.Boolean&&(bool)root["visualDesign"]!;
            var direct=root["directVegetation"]?.Type==JTokenType.Boolean&&(bool)root["directVegetation"]!;
            var rocks=root["importRocks"]?.Type==JTokenType.Boolean&&(bool)root["importRocks"]!;
            var hd=root["hdTerrain"]?.Type==JTokenType.Boolean&&(bool)root["hdTerrain"]!;
            var surfaces=root["applyGroundSurfaces"]?.Type==JTokenType.Boolean?(bool)root["applyGroundSurfaces"]!:true;
            var air=root["createAirConnections"]?.Type==JTokenType.Boolean&&(bool)root["createAirConnections"]!;
            var sea=root["createSeaConnections"]?.Type==JTokenType.Boolean&&(bool)root["createSeaConnections"]!;
            float Density(string key,bool enabled)=>root[key]?.Type is JTokenType.Float or JTokenType.Integer?Step((float)root[key]!):enabled?1:0;
            var vegetationDensity=Density("vegetationDensity",forests);var rockDensity=Density("rockDensity",rocks);
            return new NetworkPreferences(roads,rail,vegetationDensity>0,visual,direct,rockDensity>0,hd,surfaces,air,sea){VegetationDensity=vegetationDensity,RockDensity=rockDensity};
        }
        catch{return Defaults;}
    }
    public static NetworkPreferences Load(ImportLog log)
    { try{return File.Exists(FilePath)?Parse(File.ReadAllText(FilePath)):Defaults;}catch(Exception e){log.Warn("Network preferences unreadable; documented defaults used: "+e.Message);return Defaults;} }
    public void Save(ImportLog log)
    {
        try
        {
            var path=FilePath;Directory.CreateDirectory(Path.GetDirectoryName(path)!);var temporary=path+".tmp";
            File.WriteAllText(temporary,new JObject{["version"]=2,["roadLevel"]=NetworkSelection.Id(Roads),["importRail"]=Rail,["importForests"]=Forests,["visualDesign"]=VisualDesign,["directVegetation"]=DirectVegetation,["importRocks"]=Rocks,["hdTerrain"]=HdTerrain,["applyGroundSurfaces"]=Surfaces,["createAirConnections"]=AirRoutes,["createSeaConnections"]=ShipRoutes,["vegetationDensity"]=VegetationDensity,["rockDensity"]=RockDensity}.ToString(Newtonsoft.Json.Formatting.None));
            if(File.Exists(path))File.Replace(temporary,path,null);else File.Move(temporary,path);
        }
        catch(Exception e){log.Warn("Network preference save failed; current choices still apply: "+e.Message);}
    }
}
