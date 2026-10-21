using ImportEarthLocation.Processing;
using ImportEarthLocation.Core;
using Unity.Mathematics;

static class ProfileEvaluationTests
{
    public static void Run()
    {
        // Exercise both the ordinary scan and the constant-dominated indexed
        // path, including mixed chunks and an entirely constant final chunk.
        foreach(var count in new[]{17,3*SparseProfileRows.ChunkSize+13})
        {
            var rows=new SparseProfileRows(3);
            for(var i=0;i<count;i++)
            {
                var variable=i<SparseProfileRows.ChunkSize*2&&i%7==0;
                rows.Append(variable?new[]{0,1,2}:Array.Empty<int>(),
                    variable?new[]{.25,-2d,3d}:Array.Empty<double>(),variable?3:0,
                    i%11-5,.125+(i%3),i%2==0?ProfileRowKind.EqualityCost:ProfileRowKind.UpperHingeCost,
                    coalescePrevious:false);
            }
            rows.Freeze();
            if(rows.IndexedVariableRows!=(count>=SparseProfileRows.ChunkSize))
                throw new Exception("Gradient fixture did not exercise the intended row traversal");
            foreach(var x in new[]{new[]{1d,2,3},new[]{0d,0,0},new[]{-5d,.25,2}})
            foreach(var objective in new[]{false,true})
            foreach(var constants in new[]{false,true})
            foreach(var derivatives in new[]{false,true})
            {
                var expectedGradient=derivatives?new double[3]:null;
                var actualGradient=derivatives?new[]{91d,92,93}:null;
                var expected=Reference(rows,x,expectedGradient,objective,constants);
                var actual=rows.Evaluate(x,actualGradient,default,objective,constants);
                Same(actual,expected,"Objective changed while skipping irrelevant constant rows");
                if(derivatives)for(var i=0;i<3;i++)
                    Same(actualGradient![i],expectedGradient![i],"Gradient arithmetic or accumulation order changed");
            }
            using var cancelled=new CancellationTokenSource();cancelled.Cancel();
            try
            {
                rows.Evaluate(new double[3],new double[3],cancelled.Token,objective:false);
                throw new Exception("Gradient-only evaluation ignored cancellation");
            }
            catch(OperationCanceledException){}
        }
        Conditioning();
        EvaluationChunkBoundaries();
        ContactFormations();
        HingeBreakpoints();
        Console.WriteLine("Profile evaluation constant-row equivalence passed");
    }

    static void EvaluationChunkBoundaries()
    {
        foreach(var indexed in new[]{false,true})
        {
            var rows=new SparseProfileRows(4);
            for(var i=0;i<SparseProfileRows.ChunkSize-1;i++)
                rows.Append(new[]{i%4},new[]{.125},1,i%5-2,.5,
                    ProfileRowKind.EqualityCost,coalescePrevious:false);
            // Starts at term 4095, so the dot product and gradient scatter
            // must cross a primitive-array boundary instead of using a span.
            rows.Append(new[]{3,1,0,2},new[]{-2d,.25,3d,-.125},4,-1,2,
                ProfileRowKind.UpperHingeCost,coalescePrevious:false);
            var crossing=rows.Row(rows.Count-1);
            if(crossing.Offset%SparseProfileRows.ChunkSize+crossing.Count<=SparseProfileRows.ChunkSize)
                throw new Exception("Evaluation fixture omitted its term chunk crossing");
            if(indexed)for(var i=0;i<3*SparseProfileRows.ChunkSize;i++)
                rows.Append(Array.Empty<int>(),Array.Empty<double>(),0,i%3-1,.25,
                    ProfileRowKind.UpperHingeCost,coalescePrevious:false);
            rows.Freeze();
            if(rows.IndexedVariableRows!=indexed)throw new Exception("Evaluation boundary fixture used the wrong row traversal");
            foreach(var x in new[]{new[]{1d,-2,3,4},new[]{-1d,2,-3,-4},
                new[]{double.NaN,2d,3,4},new[]{double.PositiveInfinity,2d,3,4}})
            foreach(var objective in new[]{false,true})
            foreach(var constants in new[]{false,true})
            foreach(var derivatives in new[]{false,true})
            {
                var expectedGradient=derivatives?new double[4]:null;
                var actualGradient=derivatives?new[]{91d,92,93,94}:null;
                Same(rows.Evaluate(x,actualGradient,default,objective,constants),
                    Reference(rows,x,expectedGradient,objective,constants),"Term chunk crossing changed objective arithmetic");
                if(derivatives)for(var i=0;i<4;i++)Same(actualGradient![i],expectedGradient![i],
                    "Term chunk crossing changed gradient scatter");
            }
        }
    }

    static void HingeBreakpoints()
    {
        var rows=new SparseProfileRows(2);
        // Interleave positive/negative/zero hinges with a nonzero equality, so
        // skipping a hinge must preserve every contributing sum and scatter.
        foreach(var target in new[]{-2d,-0d,0d,2d})
        foreach(var weight in new[]{double.Epsilon,.5,1e300})
        {
            rows.Append(new[]{0},new[]{1d},1,target,weight,ProfileRowKind.UpperHingeCost,coalescePrevious:false);
            rows.Append(new[]{1},new[]{1d},1,1,2,ProfileRowKind.EqualityCost,coalescePrevious:false);
        }
        rows.Append(Array.Empty<int>(),Array.Empty<double>(),0,2,1e300,ProfileRowKind.UpperHingeCost,coalescePrevious:false);
        rows.Freeze();
        foreach(var value in new[]{-3d,-0d,0d,3d,double.NegativeInfinity,double.PositiveInfinity,double.NaN})
        foreach(var objective in new[]{false,true})
        foreach(var constants in new[]{false,true})
        {
            var x=new[]{value,2d};var expectedGradient=new double[2];var actualGradient=new double[2];
            var expected=Reference(rows,x,expectedGradient,objective,constants);
            var actual=rows.Evaluate(x,actualGradient,default,objective,constants);
            Same(actual,expected,"Inactive hinge shortcut changed objective arithmetic");
            for(var i=0;i<2;i++)Same(actualGradient[i],expectedGradient[i],"Inactive hinge shortcut changed gradient arithmetic");
        }
    }

    static void ContactFormations()
    {
        var first=new PreparedSegment(1,10,20,new(-100,50,0),new(100,50,0),NetworkKind.Road,"primary",2,true,false,false,0)
            {NativeWidth=12,NativeAssetKey="fixture"};
        var second=first with{WayId=2,EndNodeId=30,End=new(100,59,0),Bridge=true,Layer=1};
        var assets=new Dictionary<string,NativeAssetContract>{{"fixture",new("fixture",new[]{
            new NativeCompositionEnvelope(NativeStructureMode.Ground,0,12,0,0,0,0,4,true),
            new NativeCompositionEnvelope(NativeStructureMode.Elevated,0,12,0,0,-2,0,7,true)},Array.Empty<float>())}};
        PreparedSegment Reverse(PreparedSegment s)=>s with{StartNodeId=s.EndNodeId,EndNodeId=s.StartNodeId,
            Start=s.End,End=s.Start,Control1=s.CourseControl2,Control2=s.CourseControl1};
        foreach(var reverseFirst in new[]{false,true})foreach(var reverseSecond in new[]{false,true})
        {
            var members=new[]{reverseFirst?Reverse(first):first,reverseSecond?Reverse(second):second};
            var stored=new SparseProfileRows(8);
            var expected=NetworkPhysicalContacts.Append(members,assets,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},stored,.05,default);
            long ordinal=0;
            var measured=SparseProfileRows.ForMeasurement(8,(indices,coefficients,count,target)=>
            {
                var row=stored.Row(ordinal++);
                if(row.Count!=count||row.Target!=target)throw new Exception("Shared-node orientation changed contact inequality coverage");
                for(var j=0;j<count;j++)
                    if(indices[j]!=stored.Index(row.Offset+j)||coefficients[j]!=stored.Coefficient(row.Offset+j))
                        throw new Exception("Shared-node orientation changed canonical contact coefficients");
            });
            var actual=NetworkPhysicalContacts.Append(members,assets,new[]{0,4},new[]{1,5},new[]{2,6},new[]{3,7},measured,.05,default);
            if(stored.Count==0||ordinal!=stored.Count||actual.Rows!=expected.Rows||actual.Regions!=expected.Regions||
                actual.GradeContradictionPairs!=expected.GradeContradictionPairs)
                throw new Exception("Shared start/end contact lost its clearance coverage or classification");
        }
        var collapsed=first with{End=first.Start};var cache=new NetworkPhysicalContacts.GeometryCache();
        foreach(var reuse in new[]{false,true})
        {
            var evidence=new List<NetworkPhysicalContacts.ContactEvidence>();
            var observed=NetworkPhysicalContacts.Append(new[]{collapsed},assets,new[]{0},new[]{1},new[]{2},new[]{3},
                new SparseProfileRows(4),.05,default,evidence:evidence.Add,geometry:cache);
            var unavailable=evidence.Where(e=>e.Classification=="unavailable-footprint-leaf").ToArray();
            if(observed.UncertifiedLeaves!=1||observed.CandidateTests!=0||unavailable.Length!=1||
                unavailable[0].FirstSection!=0||unavailable[0].FirstLeaf!=0||unavailable[0].SecondSection!=-1||
                unavailable[0].FirstBegin!=0||unavailable[0].FirstEnd!=1||unavailable[0].FirstRow!=-1)
                throw new Exception("Isolated uncertified footprint disappeared from interval evidence");
            if(cache.Builds!=1||cache.Reuses!=(reuse?1:0))throw new Exception("Uncertified interval evidence bypassed geometry reuse");
        }
    }

    static void Conditioning()
    {
        var rows=new SparseProfileRows(8);
        for(var i=0;i<SparseProfileRows.ChunkSize+17;i++)
        {
            var count=i%9;
            rows.Append(Enumerable.Range(0,8).ToArray(),new[]{.25,-2d,3d,-.125,7d,1e-8,-1e4,.5},count,
                i%13,.125+i%5,i%2==0?ProfileRowKind.EqualityCost:ProfileRowKind.UpperHingeCost,coalescePrevious:false);
        }
        rows.Freeze();
        var diagonal=new double[8];var inverse=new double[8];var sums=new double[8];
        var boundary=false;
        for(long r=0;r<rows.Count;r++)
        {
            var row=rows.Row(r);boundary|=row.Offset%SparseProfileRows.ChunkSize+row.Count>SparseProfileRows.ChunkSize;
            for(var j=0;j<row.Count;j++)
            {var a=rows.Coefficient(row.Offset+j);diagonal[rows.Index(row.Offset+j)]+=row.Weight*a*a;}
        }
        for(var i=0;i<8;i++){diagonal[i]=Math.Max(1e-9,diagonal[i]);inverse[i]=1/Math.Sqrt(diagonal[i]);}
        for(long r=0;r<rows.Count;r++)
        {
            var row=rows.Row(r);var norm=0d;
            for(var j=0;j<row.Count;j++)norm+=Math.Abs(rows.Coefficient(row.Offset+j))*inverse[rows.Index(row.Offset+j)];
            for(var j=0;j<row.Count;j++)sums[rows.Index(row.Offset+j)]+=row.Weight*Math.Abs(rows.Coefficient(row.Offset+j))*inverse[rows.Index(row.Offset+j)]*norm;
        }
        if(!boundary)throw new Exception("Conditioning fixture omitted term chunk crossings");
        var lipschitz=0d;foreach(var sum in sums)lipschitz=Math.Max(lipschitz,sum);
        var actual=rows.Conditioning(default);
        for(var i=0;i<8;i++)Same(actual.D[i],diagonal[i],"Conditioning diagonal arithmetic changed");
        Same(actual.L,1.01*lipschitz,"Conditioning bound arithmetic changed");
        var empty=new SparseProfileRows(2);empty.Freeze();var emptyCondition=empty.Conditioning(default);
        if(emptyCondition.L!=0||emptyCondition.D.Any(d=>d!=1e-9))throw new Exception("Empty conditioning changed its zero-curvature metric");
        using var cancelled=new CancellationTokenSource();cancelled.Cancel();
        try{rows.Conditioning(cancelled.Token);throw new Exception("Conditioning ignored cancellation");}
        catch(OperationCanceledException){}
    }

    static double Reference(SparseProfileRows rows,double[] x,double[]? gradient,bool objective,bool constants)
    {
        var sum=0d;
        for(long i=0;i<rows.Count;i++)
        {
            var row=rows.Row(i);
            if(row.Count==0&&!constants)continue;
            var residual=-row.Target;
            for(var j=0;j<row.Count;j++)residual+=rows.Coefficient(row.Offset+j)*x[rows.Index(row.Offset+j)];
            if(row.Kind==ProfileRowKind.UpperHingeCost)residual=Math.Max(0,residual);
            if(objective)sum+=.5*row.Weight*residual*residual;
            if(gradient!=null&&residual!=0)for(var j=0;j<row.Count;j++)
                gradient[rows.Index(row.Offset+j)]+=row.Weight*residual*rows.Coefficient(row.Offset+j);
        }
        return sum;
    }

    static void Same(double actual,double expected,string message)
    {
        if(BitConverter.DoubleToInt64Bits(actual)!=BitConverter.DoubleToInt64Bits(expected))
            throw new Exception($"{message}: actual={actual:R}, expected={expected:R}");
    }
}
