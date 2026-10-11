using System;
using System.IO;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Data;

public sealed record NetworkPreferences(RoadImportLevel Roads, bool Rail, bool Forests = true)
{
    public static NetworkPreferences Defaults => new(RoadImportLevel.Primary,true);
    private static string FilePath => Path.Combine(UnityEngine.Application.persistentDataPath,"ModsSettings",Mod.Id,"network-settings.json");
    public static NetworkPreferences Parse(string json)
    {
        try
        {
            var root=JObject.Parse(json);
            if(root["version"]!=null && (root["version"]!.Type!=JTokenType.Integer || (int)root["version"]!>1))return Defaults;
            var legacy=root["includeSecondaryRoads"]?.Type==JTokenType.Boolean && (bool)root["includeSecondaryRoads"]!;
            var roads=root["roadLevel"]?.Type==JTokenType.String?NetworkSelection.Parse((string?)root["roadLevel"]):root["roadLevel"]==null&&legacy?RoadImportLevel.Secondary:RoadImportLevel.Primary;
            var rail=root["importRail"]?.Type==JTokenType.Boolean?(bool)root["importRail"]!:true;
            var forests=root["importForests"]?.Type==JTokenType.Boolean?(bool)root["importForests"]!:true;
            return new NetworkPreferences(roads,rail,forests);
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
            File.WriteAllText(temporary,new JObject{["version"]=1,["roadLevel"]=NetworkSelection.Id(Roads),["importRail"]=Rail,["importForests"]=Forests}.ToString(Newtonsoft.Json.Formatting.None));
            if(File.Exists(path))File.Replace(temporary,path,null);else File.Move(temporary,path);
        }
        catch(Exception e){log.Warn("Network preference save failed; current choices still apply: "+e.Message);}
    }
}
