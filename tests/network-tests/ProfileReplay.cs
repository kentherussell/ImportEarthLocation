using System.IO.Compression;
using ImportEarthLocation.Processing;
using Newtonsoft.Json.Linq;

static class ProfileReplay
{
    public static void SelfTest()
    {
        var root=Path.Combine(Path.GetTempPath(),"iel-profile-replay-"+Guid.NewGuid().ToString("N"));
        try
        {
            var archive=new ImportEarthLocation.Core.RunArchive(root,new{},"missing.dll");
            var records=new object[]{
                new{record="header",variables=2,rows=4,scale=20,problemScope="global-problem",retainedVectorAvailable=false},
                new{record="variable",local=0,prior=0,retained=0},new{record="variable",local=1,prior=0,retained=0},
                new{record="row",ordinal=0,indices=new[]{0,1},coefficients=new[]{1d,1d},Target=2d,Weight=1d,kind="EqualityCost"},
                new{record="row",ordinal=1,indices=new[]{0,1},coefficients=new[]{1d,-1d},Target=0d,Weight=1000d,kind="EqualityCost"},
                new{record="row",ordinal=2,indices=new[]{0},coefficients=new[]{1d},Target=0d,Weight=1e-8,kind="EqualityCost"},
                new{record="row",ordinal=3,indices=new[]{1},coefficients=new[]{1d},Target=0d,Weight=1e-8,kind="EqualityCost"},
                new{record="complete",variables=2,rows=4,terms=6,stars=0}};
            var file=archive.WriteCompressedRecords("fixture.ndjson.gz",records);
            var result=Run(Path.Combine(archive.DirectoryPath,file.File));
            if(result.Report.Outcome!=ProfileSolveOutcome.Converged||result.Heights.Any(h=>Math.Abs(h-1)>2e-4))throw new Exception("Full compressed replay lost the independently known coupled optimum.");
            var truncated=archive.WriteCompressedRecords("incomplete.ndjson.gz",records.Take(records.Length-1));var rejected=false;
            try{Run(Path.Combine(archive.DirectoryPath,truncated.File));}catch(InvalidDataException){rejected=true;}
            if(!rejected)throw new Exception("Replay certified a problem without its completeness trailer.");
            var duplicates=new object[]{
                new{record="header",variables=1,rows=2,scale=20,problemScope="global-problem",retainedVectorAvailable=false},
                new{record="variable",local=0,prior=0,retained=0},
                new{record="row",ordinal=0,indices=new[]{0},coefficients=new[]{1d},Target=2d,Weight=1d,kind="EqualityCost"},
                new{record="row",ordinal=1,indices=new[]{0},coefficients=new[]{1d},Target=2d,Weight=1d,kind="EqualityCost"},
                new{record="complete",variables=1,rows=2,terms=2,stars=0}};
            var duplicateFile=archive.WriteCompressedRecords("adjacent-rows.ndjson.gz",duplicates);
            var duplicateResult=Run(Path.Combine(archive.DirectoryPath,duplicateFile.File));
            if(duplicateResult.Report.Outcome!=ProfileSolveOutcome.Converged||Math.Abs(duplicateResult.Heights[0]-2)>1e-7)
                throw new Exception("Replay coalesced archived rows before verifying their ordinals and completeness.");
            Console.WriteLine("PASS: complete compressed global-problem replay reaches its analytical optimum; incomplete mathematical input is refused.");
        }
        finally{if(Directory.Exists(root))Directory.Delete(root,true);}
    }
    public static ProfileSolveResult Run(string path)=>RunCore(path,null)!;
    public static void BenchmarkKernels(string path,string output)=>RunCore(path,output);
    public static void BenchmarkEvaluation(string path,string output)=>RunCore(path,null,output);
    public static void Inspect(string path,Action<SparseProfileRows,double[],IReadOnlyList<ProfileStar>,double> inspect)
        =>RunCore(path,null,inspect:inspect);
    private static ProfileSolveResult? RunCore(string path,string? kernelReport,string? evaluationReport=null,
        Action<SparseProfileRows,double[],IReadOnlyList<ProfileStar>,double>? inspect=null)
    {
        using var file=File.OpenRead(path);using var gzip=new GZipStream(file,CompressionMode.Decompress);using var reader=new StreamReader(gzip);
        SparseProfileRows? rows=null;double[]? prior=null;double[]? retained=null;double scale=0;long expectedRows=0;var stars=new List<ProfileStar>();
        var variablesSeen=new HashSet<int>();var complete=false;var globalProblem=false;var capturedVectorAvailable=false;
        while(reader.ReadLine() is {} line)
        {
            var record=JObject.Parse(line);
            switch((string?)record["record"])
            {
                case "header":
                    if(rows!=null)throw new InvalidDataException("Duplicate replay header.");
                    if((string?)record["problemScope"]=="result-only")throw new InvalidDataException("Result-only capture; replay profile-problem.ndjson.gz instead.");
                    globalProblem=(string?)record["problemScope"]=="global-problem";capturedVectorAvailable=(bool?)record["retainedVectorAvailable"]??true;
                    var count=(int)record["variables"]!;expectedRows=(long)record["rows"]!;scale=(double)record["scale"]!;
                    rows=new(count);prior=new double[count];retained=new double[count];break;
                case "variable":
                    var index=(int)record["local"]!;
                    if(prior==null||!variablesSeen.Add(index)||index<0||index>=prior.Length)throw new InvalidDataException("Invalid/duplicate replay variable.");
                    prior[index]=(double)record["prior"]!;retained![index]=(double)record["retained"]!;break;
                case "star":
                    stars.Add(new((int)record["Height"]!,record["Controls"]!.ToObject<int[]>()!,record["Alpha"]!.ToObject<double[]>()!,
                        record["Groups"]!.ToObject<int[]>()!,record["FixedHeight"]!.Type==JTokenType.Null?null:(double?)record["FixedHeight"],record["FixedSlopes"]!.ToObject<double?[]>()));break;
                case "row":
                    if(rows==null||(long)record["ordinal"]! != rows.Count)throw new InvalidDataException("Missing or out-of-order replay row.");
                    var indices=record["indices"]!.ToObject<int[]>()!;
                    rows.Append(indices,record["coefficients"]!.ToObject<double[]>()!,indices.Length,(double)record["Target"]!,
                        (double)record["Weight"]!,Enum.Parse<ProfileRowKind>((string)record["kind"]!),coalescePrevious:false);break;
                case "complete":
                    if(rows==null||(long)record["rows"]! != rows.Count||(long)record["terms"]! !=rows.Terms||(int)record["stars"]! !=stars.Count)
                        throw new InvalidDataException("Replay trailer does not match its records.");
                    complete=true;break;
                default:throw new InvalidDataException("Unknown replay record.");
            }
        }
        if(!complete||rows==null||prior==null||variablesSeen.Count!=prior.Length||rows.Count!=expectedRows)
            throw new InvalidDataException("Replay incomplete; do not benchmark or certify a truncated fixture.");
        rows.Freeze();
        if(inspect!=null){inspect(rows,prior,stars,scale);return null;}
        var watch=System.Diagnostics.Stopwatch.StartNew();var capturedObjective=rows.Evaluate(retained!,null,default);
        if(evaluationReport!=null)
        {
            var gradient=new double[prior.Length];var expectedGradient=new double[prior.Length];
            var expected=rows.Evaluate(prior,expectedGradient,default,includeConstantObjective:false);
            long hingeRows=0,inactiveHinges=0,constantRows=0;
            for(long r=0;r<rows.Count;r++)
            {
                var row=rows.Row(r);if(row.Count==0)constantRows++;
                if(row.Kind!=ProfileRowKind.UpperHingeCost)continue;
                hingeRows++;var residual=-row.Target;
                for(var j=0;j<row.Count;j++)residual+=rows.Coefficient(row.Offset+j)*prior[rows.Index(row.Offset+j)];
                if(residual<=0)inactiveHinges++;
            }
            var timings=new List<double>();var consumed=0d;
            for(var batch=-1;batch<7;batch++)
            {
                var timer=System.Diagnostics.Stopwatch.StartNew();
                for(var sweep=0;sweep<20;sweep++)consumed+=rows.Evaluate(prior,gradient,default,includeConstantObjective:false);
                if(batch>=0)timings.Add(timer.Elapsed.TotalSeconds);
                if(rows.Evaluate(prior,gradient,default,includeConstantObjective:false)!=expected||!gradient.SequenceEqual(expectedGradient))
                    throw new Exception("Evaluation changed between identical sweeps.");
            }
            using var sha=System.Security.Cryptography.SHA256.Create();var gradientBytes=new byte[gradient.Length*sizeof(double)];
            Buffer.BlockCopy(gradient,0,gradientBytes,0,gradientBytes.Length);
            File.WriteAllText(evaluationReport,Newtonsoft.Json.JsonConvert.SerializeObject(new {source=Path.GetFileName(path),rows=rows.Count,terms=rows.Terms,
                variables=rows.Variables,objective=expected,gradientSha256=Convert.ToHexString(sha.ComputeHash(gradientBytes)),
                assemblyMvid=typeof(NetworkProfileSolve).Assembly.ManifestModule.ModuleVersionId,secondsPer20Sweeps=timings,
                hingeRows,inactiveHingesAtPrior=inactiveHinges,constantRows,
                medianSeconds=timings.OrderBy(t=>t).ElementAt(3),consumed,scope="objective-plus-gradient-at-complete-captured-prior",
                loadingAndConditioningExcluded=true,wholeDesignSpeedupVerified=false},Newtonsoft.Json.Formatting.Indented));
            Console.WriteLine("Evaluation kernel report: "+evaluationReport);return null;
        }
        if(kernelReport!=null)
        {
            var gradient=new double[prior.Length];rows.Evaluate(prior,gradient,default);
            var conditioning=rows.Conditioning(default);var scratch=new double[prior.Length];
            double Reference()
            {
                var denominator=0d;for(var j=0;j<gradient.Length;j++)denominator+=gradient[j]*(gradient[j]/conditioning.D[j]);
                if(!(denominator>0)||!SparseProfileRows.Finite(denominator))return conditioning.L;
                var numerator=0d;
                for(long r=0;r<rows.Count;r++)
                {
                    var row=rows.Row(r);var directional=0d;
                    for(var j=0;j<row.Count;j++){var v=rows.Index(row.Offset+j);directional+=rows.Coefficient(row.Offset+j)*(gradient[v]/conditioning.D[v]);}
                    numerator+=row.Weight*directional*directional;
                }
                var local=1.01*(numerator/denominator);
                return SparseProfileRows.Finite(local)&&local>0?Math.Min(conditioning.L,local):conditioning.L;
            }
            double Optimized()=>rows.DirectionalLipschitz(gradient,conditioning.D,conditioning.L,scratch,default);
            var expected=Reference();var actual=Optimized();
            if(BitConverter.DoubleToInt64Bits(expected)!=BitConverter.DoubleToInt64Bits(actual))throw new Exception("Captured directional curvature changed its arithmetic result");
            var referenceTimes=new List<double>();var optimizedTimes=new List<double>();
            void Measure(Func<double> kernel,List<double> timings)
            {var timer=System.Diagnostics.Stopwatch.StartNew();var value=kernel();timings.Add(timer.Elapsed.TotalSeconds);if(value!=expected)throw new Exception("Directional result changed between timing sweeps");}
            for(var i=0;i<6;i++)
            {
                if(i%2==0){Measure(Reference,referenceTimes);Measure(Optimized,optimizedTimes);}
                else{Measure(Optimized,optimizedTimes);Measure(Reference,referenceTimes);}
            }
            File.WriteAllText(kernelReport,Newtonsoft.Json.JsonConvert.SerializeObject(new{source=Path.GetFileName(path),variables=prior.Length,rows=rows.Count,terms=rows.Terms,
                scope="directional-curvature-only-at-captured-prior",completeInputVerified=true,bitIdenticalResult=true,result=actual,
                referenceSeconds=referenceTimes,optimizedSeconds=optimizedTimes,alternatingOrder=true,warmupExcluded=true,
                matrixLoadingAndConditioningExcluded=true,wholeSolverSpeedupMeasured=false},Newtonsoft.Json.Formatting.Indented));
            Console.WriteLine("Captured kernel report saved: "+kernelReport);
            return null;
        }
        var result=globalProblem?NetworkProfileSolve.SolveComponents(rows,prior,scale,stars.ToArray(),diagnostic:Console.WriteLine):
            NetworkProfileSolve.SolveComponent(rows,prior,new(prior.Length,scale,stars.ToArray()),diagnostic:Console.WriteLine);
        Console.WriteLine(Newtonsoft.Json.JsonConvert.SerializeObject(new{source=Path.GetFileName(path),variables=prior.Length,rows=rows.Count,terms=rows.Terms,
            capturedObjective,capturedVectorAvailable,replayedObjective=result.Report.Objective,elapsedSeconds=watch.Elapsed.TotalSeconds,result.Report,
            sameFrozenMathematicalProblem=true,nativeRenderingVerified=false}));
        return result;
    }
}
