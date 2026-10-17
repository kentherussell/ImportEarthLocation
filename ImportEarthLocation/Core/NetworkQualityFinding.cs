namespace ImportEarthLocation.Core;

public sealed record NetworkQualityFinding(string Stage,string Category,string Scope,int Checked,int Failed,int Unavailable,
    double Worst,double AffectedLengthMeters,string Evidence)
{
    public string State=>Failed>0?"Failed":Unavailable>0?"Unavailable":Checked==0?"NotApplicable":"Passed";
}
