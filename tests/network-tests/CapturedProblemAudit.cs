using System.IO.Compression;
using ImportEarthLocation.Processing;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

// Independent streaming evaluation of the captured RETURNED vector. This is
// deliberately not an optimizer replay and retains no city-sized row matrix.
static class CapturedProblemAudit
{
    static IEnumerable<JObject> Records(string path)
    {
        using var file=File.OpenRead(path);using var gzip=new GZipStream(file,CompressionMode.Decompress);
        using var reader=new StreamReader(gzip);
        while(reader.ReadLine() is {} line)yield return JObject.Parse(line);
    }
    static string Artifact(string folder,string logical)
        =>CapturedRunFiles.Resolve(folder,logical);
    public static void SelfTest()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-returned-audit-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(root);
        try
        {
            var archive=new ImportEarthLocation.Core.RunArchive(root,new{},"missing.dll");
            var variables=new object[]{new{record="variable",local=0,prior=0,retained=1},new{record="variable",local=1,prior=0,retained=2}};
            archive.WriteCompressedRecords("profile-result.ndjson.gz",new object[]{new{record="header",variables=2,retainedVectorAvailable=true}}
                .Concat(variables).Append(new{record="complete",variables=2,rows=0,terms=0,stars=0}));
            var problem=new object[]{new{record="header",variables=2,rows=2,problemScope="global-problem"}}
                .Concat(variables).Concat(new object[]{
                    new{record="row",ordinal=0,indices=new[]{0,1},coefficients=new[]{1d,1d},Target=4d,Weight=2d,kind="EqualityCost"},
                    new{record="row",ordinal=1,indices=Array.Empty<int>(),coefficients=Array.Empty<double>(),Target=-2d,Weight=3d,kind="UpperHingeCost"},
                    new{record="complete",variables=2,rows=2,terms=2,stars=0}}).ToArray();
            archive.WriteCompressedRecords("profile-problem.ndjson.gz",problem);
            var output=Path.Combine(root,"report.json");Run(archive.DirectoryPath,output);var result=JObject.Parse(File.ReadAllText(output));
            if((double)result["objective"]! !=7||(long)result["constantConflicts"]! !=1)throw new Exception("Independent captured audit lost known objective/constant conflict");
            archive.WriteCompressedRecords("profile-problem.ndjson.gz",problem.Take(problem.Length-1));var rejected=false;
            try{Run(archive.DirectoryPath,output);}catch(InvalidDataException){rejected=true;}
            if(!rejected)throw new Exception("Independent captured audit accepted missing completeness trailer");
            Console.WriteLine("PASS: independent returned-vector objective, constant contradictions and incomplete-input rejection");
        }
        finally{Directory.Delete(root,true);}
    }
    public static void Run(string folder,string output)
    {
        var clock=System.Diagnostics.Stopwatch.StartNew();double[]? vector=null;var seen=new HashSet<int>();var complete=false;
        double? capturedObjective=null;var scale=0d;var hardResidual=0d;
        foreach(var r in Records(Artifact(folder,"profile-result.ndjson.gz")))
        {
            if(complete)throw new InvalidDataException("Records after result trailer");
            switch((string?)r["record"])
            {
                case "header":
                    if(vector!=null||(bool?)r["retainedVectorAvailable"]!=true)throw new InvalidDataException("Missing/duplicate returned-vector header");
                    vector=new double[(int)r["variables"]!];capturedObjective=(double?)r["report"]?["Objective"];break;
                case "variable":
                    var i=(int)r["local"]!;
                    if(vector==null||i<0||i>=vector.Length||!seen.Add(i))throw new InvalidDataException("Invalid returned-vector identity");
                    vector[i]=(double)r["retained"]!;
                    if(!double.IsFinite(vector[i]))throw new InvalidDataException("Nonfinite returned height");break;
                case "complete":
                    if(vector==null||seen.Count!=vector.Length||(int)r["variables"]! !=vector.Length)throw new InvalidDataException("Incomplete returned vector");
                    complete=true;break;
                case "star":break;
                default:throw new InvalidDataException("Unexpected result record");
            }
        }
        if(!complete||vector==null)throw new InvalidDataException("Returned result lacks a completeness trailer");
        var diagonal=new double[vector.Length];var kinds=new Dictionary<string,(long Rows,long Active,double Objective,double Worst)>();
        long rows=0,terms=0,constantConflicts=0,expectedRows=-1;var stars=0;complete=false;var variables=new HashSet<int>();
        var witnesses=new List<object>();var minimumCoefficient=double.PositiveInfinity;var maximumCoefficient=0d;var objective=0d;
        var largestCosts=new List<(double Value,object Row)>();var stiffestRows=new List<(double Value,object Row)>();
        static void Retain(List<(double Value,object Row)> list,double value,object row)
        {
            if(list.Count==16&&value<=list[list.Count-1].Value)return;
            list.Add((value,row));list.Sort((a,b)=>b.Value.CompareTo(a.Value));if(list.Count>16)list.RemoveAt(16);
        }
        foreach(var r in Records(Artifact(folder,"profile-problem.ndjson.gz")))
        {
            if(complete)throw new InvalidDataException("Records after problem trailer");
            switch((string?)r["record"])
            {
                case "header":
                    if(expectedRows>=0||(int)r["variables"]! !=vector.Length||(string?)r["problemScope"]!="global-problem")throw new InvalidDataException("Mismatched problem header");
                    expectedRows=(long)r["rows"]!;scale=(double?)r["scale"]??0;break;
                case "variable":
                    var variable=(int)r["local"]!;
                    if(variable<0||variable>=vector.Length||!variables.Add(variable))throw new InvalidDataException("Invalid problem variable");break;
                case "star":
                    var height=(int)r["Height"]!;var controls=r["Controls"]!.ToObject<int[]>()!;
                    var alpha=r["Alpha"]!.ToObject<double[]>()!;var groups=r["Groups"]!.ToObject<int[]>()!;
                    if(height<0||height>=vector.Length||controls.Length!=alpha.Length||controls.Length!=groups.Length)throw new InvalidDataException("Invalid star identity");
                    var h=vector[height];var first=new Dictionary<int,int>();
                    if(r["FixedHeight"]?.Type!=JTokenType.Null&&r["FixedHeight"]!=null)hardResidual=Math.Max(hardResidual,Math.Abs(h-(double)r["FixedHeight"]!));
                    var fixedSlopes=r["FixedSlopes"]?.ToObject<double?[]>();
                    for(var j=0;j<controls.Length;j++)
                    {
                        if(controls[j]<0||controls[j]>=vector.Length||!double.IsFinite(alpha[j])||alpha[j]==0)throw new InvalidDataException("Invalid star control");
                        var delta=vector[controls[j]]-h;
                        if(first.TryGetValue(groups[j],out var k))
                        {
                            var denominator=Math.Max(Math.Abs(alpha[k]),Math.Abs(alpha[j]));
                            hardResidual=Math.Max(hardResidual,Math.Abs(delta*(alpha[k]/denominator)-(vector[controls[k]]-h)*(alpha[j]/denominator)));
                        }
                        else first[groups[j]]=j;
                        if(fixedSlopes!=null&&groups[j]>=0&&groups[j]<fixedSlopes.Length&&fixedSlopes[groups[j]].HasValue)
                            hardResidual=Math.Max(hardResidual,Math.Abs(delta-alpha[j]*fixedSlopes[groups[j]]!.Value));
                    }
                    stars++;break;
                case "row":
                    if((long)r["ordinal"]! !=rows)throw new InvalidDataException("Missing or duplicate problem row");
                    var indices=r["indices"]!.ToObject<int[]>()!;var a=r["coefficients"]!.ToObject<double[]>()!;
                    var target=(double)r["Target"]!;var weight=(double)r["Weight"]!;var kind=(string)r["kind"]!;
                    if(indices.Length!=a.Length||!double.IsFinite(target)||!double.IsFinite(weight)||weight<0)throw new InvalidDataException("Invalid row");
                    var residual=-target;var stiffness=0d;
                    for(var j=0;j<a.Length;j++)
                    {
                        if(indices[j]<0||indices[j]>=vector.Length||!double.IsFinite(a[j]))throw new InvalidDataException("Invalid coefficient");
                        residual+=a[j]*vector[indices[j]];diagonal[indices[j]]+=weight*a[j]*a[j];
                        stiffness+=weight*a[j]*a[j];
                        if(a[j]!=0){minimumCoefficient=Math.Min(minimumCoefficient,Math.Abs(a[j]));maximumCoefficient=Math.Max(maximumCoefficient,Math.Abs(a[j]));}
                    }
                    if(kind==nameof(ProfileRowKind.UpperHingeCost))residual=Math.Max(0,residual);
                    else if(kind!=nameof(ProfileRowKind.EqualityCost))throw new InvalidDataException("Unknown row kind");
                    var cost=.5*weight*residual*residual;objective+=cost;kinds.TryGetValue(kind,out var old);
                    if(largestCosts.Count<16||cost>largestCosts[largestCosts.Count-1].Value||stiffestRows.Count<16||stiffness>stiffestRows[stiffestRows.Count-1].Value)
                    {
                        var witness=new{row=rows,indices,coefficients=a,target,weight,kind,residual,cost,stiffness,retainedHeights=indices.Select(i=>vector[i]).ToArray()};
                        Retain(largestCosts,cost,witness);Retain(stiffestRows,stiffness,witness);
                    }
                    kinds[kind]=(old.Rows+1,old.Active+(residual!=0?1:0),old.Objective+cost,Math.Max(old.Worst,Math.Abs(residual)));
                    if(indices.Length==0&&residual!=0&&weight>0)
                    {constantConflicts++;if(witnesses.Count<32)witnesses.Add(new{row=rows,target,weight,kind,residual,objective=cost});}
                    rows++;terms+=indices.Length;break;
                case "complete":
                    if(rows!=expectedRows||(long)r["rows"]! !=rows||(long)r["terms"]! !=terms||(int)r["stars"]! !=stars||variables.Count!=vector.Length)
                        throw new InvalidDataException("Problem completeness mismatch");complete=true;break;
                default:throw new InvalidDataException("Unknown problem record");
            }
        }
        if(!complete)throw new InvalidDataException("Problem lacks a completeness trailer");
        var positive=diagonal.Where(d=>d>0).ToArray();
        File.WriteAllText(output,JsonConvert.SerializeObject(new{capture=Path.GetFileName(folder),variables=vector.Length,rows,terms,stars,
            objective,capturedObjective,objectiveMatchesCapturedReport=capturedObjective.HasValue?(bool?)(Math.Abs(objective-capturedObjective.Value)<=1e-8*Math.Max(1,Math.Abs(capturedObjective.Value))):null,
            maximumHardStarResidualMeters=hardResidual,maximumBoxViolationMeters=scale>0?(double?)vector.Max(x=>Math.Max(0,Math.Max(-x,x-scale))):null,
            constantConflicts,constantWitnesses=witnesses,largestCosts=largestCosts.Select(r=>r.Row),stiffestRows=stiffestRows.Select(r=>r.Row),minimumCoefficient,maximumCoefficient,
            diagonalMinimum=positive.Length>0?positive.Min():0,diagonalMaximum=positive.Length>0?positive.Max():0,
            diagonalDynamicRangeIsNotConditionNumber=true,rowsByKind=kinds.Select(k=>new{kind=k.Key,k.Value.Rows,k.Value.Active,k.Value.Objective,k.Value.Worst}),
            elapsedSeconds=clock.Elapsed.TotalSeconds,scope="exact-captured-problem-at-captured-returned-vector",physicalMetersNotInferredFromNormalizedResiduals=true,
            solverRerun=false,nativeOutcomePredicted=false},Formatting.Indented));
        Console.WriteLine("Captured returned-vector audit saved: "+output);
    }
}
