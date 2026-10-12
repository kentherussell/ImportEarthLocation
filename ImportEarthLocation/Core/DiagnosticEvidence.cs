using Newtonsoft.Json.Linq;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

public static class DiagnosticEvidence
{
    public static bool AllExpectedPresent(JArray expected,JArray? observed)
    {
        if(expected.Count==0||observed==null)return false;
        var native=new HashSet<JToken>(observed,new JTokenEqualityComparer());
        foreach(var item in expected)if(!native.Contains(item))return false;
        return true;
    }

    public static string Compare(bool applied, JToken? baseline, JToken? current)
    {
        if(!applied||baseline==null||current==null||baseline.Type==JTokenType.Null||current.Type==JTokenType.Null)return "unverified";
        return JToken.DeepEquals(baseline,current)?"matching-native-state":"native-state-differs";
    }
}
