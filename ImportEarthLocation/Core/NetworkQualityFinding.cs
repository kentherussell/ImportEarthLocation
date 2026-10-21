namespace ImportEarthLocation.Core;

public sealed record NetworkQualityFinding(string Stage,string Category,string Scope,int Checked,int Failed,int Unavailable,
    double Worst,double AffectedLengthMeters,string Evidence)
{
    public int Expected { get; init; }=Checked+Unavailable;
    public int IntentionallyExcluded { get; init; }
    public int NotApplicable { get; init; }
    public string State=>Failed>0?"Failed":Unavailable>0||Expected>Checked+IntentionallyExcluded+NotApplicable?"Unavailable":Checked==0?"NotApplicable":"Passed";
}
