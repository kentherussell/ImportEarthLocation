using ImportEarthLocation.Processing;

static class ProfileMathTests
{
    static void Check(bool condition,string message){if(!condition)throw new Exception(message);}
    static void Close(double actual,double expected,double epsilon,string message)=>Check(Math.Abs(actual-expected)<=epsilon,$"{message}: actual={actual:R}, expected={expected:R}");
    static SparseProfileRows Coupled(double strength)
    {
        var rows=new SparseProfileRows(2);
        rows.Append(new[]{0,1},new[]{1d,1d},2,2,1,ProfileRowKind.EqualityCost);
        rows.Append(new[]{0,1},new[]{1d,-1d},2,0,strength,ProfileRowKind.EqualityCost);
        rows.Append(new[]{0},new[]{1d},1,0,1e-8,ProfileRowKind.EqualityCost);
        rows.Append(new[]{1},new[]{1d},1,0,1e-8,ProfileRowKind.EqualityCost);
        rows.Freeze();return rows;
    }
    public static void Run()
    {
        ProfileReplay.SelfTest();
        var chunked=new SparseProfileRows(8);var expectedObjective=0d;var expectedGradient=new double[8];
        var chunkPoint=Enumerable.Range(0,8).Select(i=>i*.7).ToArray();
        for(var r=0;r<9000;r++)
        {
            var count=1+r%8;var ii=Enumerable.Range(0,count).ToArray();var aa=ii.Select(j=>(j+1)*.13).ToArray();
            var target=r%11-5d;var weight=1+(r%3)*.2;var kind=r%2==0?ProfileRowKind.EqualityCost:ProfileRowKind.UpperHingeCost;
            chunked.Append(ii,aa,count,target,weight,kind);
            var residual=-target;for(var j=0;j<count;j++)residual+=aa[j]*chunkPoint[j];
            if(kind==ProfileRowKind.UpperHingeCost)residual=Math.Max(0,residual);
            expectedObjective+=.5*weight*residual*residual;
            for(var j=0;j<count;j++)expectedGradient[j]+=weight*residual*aa[j];
        }
        chunked.Freeze();var chunkGradient=new double[8];
        Close(chunked.Evaluate(chunkPoint,chunkGradient,default),expectedObjective,0,"Chunk-local objective changed accumulation");
        for(var j=0;j<8;j++)Close(chunkGradient[j],expectedGradient[j],0,"Chunk boundary gradient changed accumulation");
        var metric=Enumerable.Range(0,8).Select(i=>1d+i*.3).ToArray();var numerator=0d;var denominator=0d;
        for(var j=0;j<8;j++)denominator+=chunkGradient[j]*(chunkGradient[j]/metric[j]);
        for(long r=0;r<chunked.Count;r++)
        {
            var row=chunked.Row(r);var directional=0d;
            for(var j=0;j<row.Count;j++){var v=chunked.Index(row.Offset+j);directional+=chunked.Coefficient(row.Offset+j)*(chunkGradient[v]/metric[v]);}
            numerator+=row.Weight*directional*directional;
        }
        Close(chunked.DirectionalLipschitz(chunkGradient,metric,double.MaxValue,new double[8],default),1.01*(numerator/denominator),0,
            "Chunk-local directional curvature changed arithmetic across sparse chunk boundaries");
        var rows=new SparseProfileRows(3);
        rows.Append(new[]{0,1,0},new[]{1d,2d,3d},3,5,2,ProfileRowKind.EqualityCost);
        rows.Append(new[]{1,2},new[]{2d,-1d},2,1,3,ProfileRowKind.UpperHingeCost);
        rows.Freeze();Check(rows.Terms==4,"Duplicate row terms not coalesced");
        var point=new[]{1d,2d,3d};var gradient=new double[3];var f=rows.Evaluate(point,gradient,default);
        // Equality residual=3 -> objective9; hinge residual=0 -> objective0.
        Close(f,9,1e-12,"Analytical objective");Close(gradient[0],24,1e-12,"Coalesced analytical gradient");
        foreach(var x in new[]{new[]{1d,2d,3d},new[]{1d,3d,2d}})
        {
            rows.Evaluate(x,gradient,default);
            for(var j=0;j<3;j++)
            {var epsilon=1e-6;var old=x[j];x[j]=old+epsilon;var plus=rows.Evaluate(x,null,default);x[j]=old-epsilon;var minus=rows.Evaluate(x,null,default);x[j]=old;Close(gradient[j],(plus-minus)/(2*epsilon),1e-5,"Finite-difference gradient");}
        }
        var star=new ProfileStar(0,new[]{1,2},new[]{-10d,10d},new[]{0,0});
        var projector=new ProfileHardProjector(3,20,new[]{star});var values=new[]{2d,15d,-5d};
        Check(projector.Project(values,new[]{1d,1d,1d}),"Box-active star projection failed");
        Close(values[0],6.4,1e-10,"Analytical active-box height");Close(values[1],12.8,1e-10,"Analytical active-box control");Close(values[2],0,1e-10,"Active lower control");
        Check(projector.TryLinearMinimum(new[]{4.4,-2.2,5d},out var support),"Star linear optimality oracle unavailable");
        Close(support,0,1e-10,"Independent KKT normal has zero linear minimum");
        Check(projector.TryLinearMinimum(new[]{-1d,-1d,-1d},out support),"Star linear extreme unavailable");
        Close(support,-60,1e-10,"All-three-height upper vertex linear minimum");
        var ports=Enumerable.Range(1,10).ToArray();var alpha=new[]{-1d,1d,-2d,2d,-3d,3d,-4d,4d,-5d,5d};
        var highDegree=new ProfileHardProjector(11,20,new[]{new ProfileStar(0,ports,alpha,new int[10])});
        var highValues=new double[11];for(var i=0;i<10;i++)highValues[i+1]=10+.2*alpha[i];
        Check(highDegree.Project(highValues,Enumerable.Repeat(1d,11).ToArray()),"High-degree star projection failed");
        Close(highValues[0],100/11d,1e-7,"Analytical high-degree mean height");
        for(var i=0;i<10;i++)Close(highValues[i+1],100/11d+.2*alpha[i],1e-7,"Analytical high-degree control");
        var fixedProjector=new ProfileHardProjector(3,20,new[]{new ProfileStar(0,new[]{1,2},new[]{-10d,10d},new[]{0,0},fixedHeight:10,fixedSlopes:new double?[]{.1})});
        var fixedValues=new[]{0d,0d,0d};Check(fixedProjector.Project(fixedValues,new[]{1d,1d,1d}),"Fixed boundaries unavailable");
        Close(fixedValues[1],9,0,"Fixed physical derivative");Close(fixedValues[2],11,0,"Fixed physical derivative");
        Check(fixedProjector.TryLinearMinimum(new[]{1d,2d,3d},out support),"Fixed linear support unavailable");
        Close(support,61,1e-12,"Fixed star is a singleton, not a box relaxation");
        var regular=Coupled(1000);var condition=regular.Conditioning(default);
        // Exact transformed eigenvalues: (2+r)/(w+1+r), (2*w+r)/(w+1+r), r=1e-8.
        Check(condition.L>=2000/1001d,"Lipschitz bound below analytical largest eigenvalue");
        var result=NetworkProfileSolve.SolveComponent(regular,new[]{0d,0d},new ProfileHardProjector(2,20,Array.Empty<ProfileStar>()));
        Console.WriteLine($"Ordinary solve: outcome={result.Report.Outcome}; iterations={result.Report.AcceptedIterations}; objective={result.Report.Objective:R}; gap={result.Report.OptimalityGap:R}; x={result.Heights[0]:R}; restarts={result.Report.Restarts}.");
        Check(result.Report.Outcome==ProfileSolveOutcome.Converged,"Ordinary coupled solve did not converge");
        Close(result.Heights[0],1,2e-4,"Analytical coupled optimum x");Close(result.Heights[1],1,2e-4,"Analytical coupled optimum y");
        Check(result.Report.AcceptedIterations>=50&&result.Report.AcceptedIterations<=2000,"Stopping window/cap invalid");
        Check(result.Report.Trials<=34*result.Report.AcceptedIterations,"Bounded majorization/restart cost exceeded");
        var asymmetric=new SparseProfileRows(2);
        asymmetric.Append(new[]{0,1},new[]{1d,2d},2,3,1,ProfileRowKind.EqualityCost);
        asymmetric.Append(new[]{0,1},new[]{1d,-1d},2,0,1000,ProfileRowKind.EqualityCost);
        asymmetric.Append(new[]{0},new[]{1d},1,0,1e-8,ProfileRowKind.EqualityCost);
        asymmetric.Append(new[]{1},new[]{1d},1,0,1e-8,ProfileRowKind.EqualityCost);asymmetric.Freeze();
        var restarted=NetworkProfileSolve.SolveComponent(asymmetric,new[]{0d,0d},new ProfileHardProjector(2,20,Array.Empty<ProfileStar>()));
        Console.WriteLine($"Asymmetric solve: outcome={restarted.Report.Outcome}; iterations={restarted.Report.AcceptedIterations}; restarts={restarted.Report.Restarts}; backtracks={restarted.Report.Backtracks}; gap={restarted.Report.OptimalityGap:R}.");
        var determinant=9000+1e-8*2005+1e-16;
        var bestGradient=new double[2];asymmetric.Evaluate(restarted.Heights,bestGradient,default);Console.WriteLine($"Asymmetric returned-best gradient=({bestGradient[0]:R},{bestGradient[1]:R}); heights=({restarted.Heights[0]:R},{restarted.Heights[1]:R}).");
        Close(restarted.Heights[0],(9000+3e-8)/determinant,2e-4,"Independent asymmetric optimum x");
        Close(restarted.Heights[1],(9000+6e-8)/determinant,2e-4,"Independent asymmetric optimum y");
        Check(restarted.Report.Outcome==ProfileSolveOutcome.Converged,"Asymmetric known optimum not certified within cap");
        Check(restarted.Report.Restarts>0,"Dedicated asymmetric restart fixture not exercised");
        // The common-height gradient proposes a low-curvature step, but fixing
        // x=0 rotates the projected displacement into the stiff difference mode.
        // This MUST reject that proposal, not assume Rayleigh is a global bound.
        var fixedCommon=new ProfileHardProjector(2,20,new[]{new ProfileStar(0,
            Array.Empty<int>(),Array.Empty<double>(),Array.Empty<int>(),fixedHeight:0)});
        var boundedTrial=NetworkProfileSolve.SolveComponent(regular,new[]{0d,0d},fixedCommon);
        Console.WriteLine($"Projected directional trial: outcome={boundedTrial.Report.Outcome}; iterations={boundedTrial.Report.AcceptedIterations}; backtracks={boundedTrial.Report.Backtracks}; y={boundedTrial.Heights[1]:R}.");
        Check(boundedTrial.Report.Backtracks>0,"Projected direction did not exercise majorization rejection");
        Check(boundedTrial.Report.Outcome==ProfileSolveOutcome.Converged,"Fixed-height quadratic did not converge");
        Close(boundedTrial.Heights[0],0,0,"Fixed native boundary moved");
        Close(boundedTrial.Heights[1],2/(1001+1e-8),1e-7,"Independent fixed-height optimum");
        Check(boundedTrial.Report.Trials<=34*boundedTrial.Report.AcceptedIterations,"Backtracking exceeded bounded trial count");
        Console.WriteLine($"PASS V-PROFILE basic mathematics: gradients, sparse coalescing, analytical projection, fixed boundaries, Lipschitz bound, FISTA restart/stopping; iterations={result.Report.AcceptedIterations}, restarts={result.Report.Restarts}.");
        // Same permitted convex formulation, but a slow common-height mode.
        // Independent optimum is (2/(2+r),2/(2+r)), r=1e-8; no second optimizer.
        var stiff=Coupled(1e12);
        var stiffResult=NetworkProfileSolve.SolveComponent(stiff,new[]{0d,0d},new ProfileHardProjector(2,20,Array.Empty<ProfileStar>()));
        var optimum=2/(2+1e-8);var optimumObjective=2e-8/(2+1e-8);
        Console.WriteLine($"V-PROFILE stiff result: outcome={stiffResult.Report.Outcome}; iterations={stiffResult.Report.AcceptedIterations}; x={stiffResult.Heights[0]:R}; y={stiffResult.Heights[1]:R}; objective={stiffResult.Report.Objective:R}; projectedStep={stiffResult.Report.ProjectedStep:R}; analyticalOptimum=({optimum:R},{optimum:R}), analyticalObjective={optimumObjective:R}.");
        Check(stiffResult.Report.Outcome==ProfileSolveOutcome.Converged&&
            (Math.Abs(stiffResult.Heights[0]-optimum)<.0005&&Math.Abs(stiffResult.Heights[1]-optimum)<.0005),
            "The prescribed stopping criteria declare convergence far from the independently known optimum under poor conditioning.");
        Console.WriteLine("PASS V-PROFILE poor-conditioning analytical convergence check.");
        var trace=new List<string>();
        var traced=NetworkProfileSolve.SolveComponent(stiff,new[]{0d,0d},new ProfileHardProjector(2,20,Array.Empty<ProfileStar>()),diagnostic:trace.Add);
        Check(trace.Any(s=>s.Contains("conditioning-start"))&&trace.Any(s=>s.Contains("conditioning-complete"))&&trace.Any(s=>s.Contains("component-complete")),"Profile diagnostic stage evidence missing");
        Check(traced.Report==stiffResult.Report&&traced.Heights.SequenceEqual(stiffResult.Heights),"Diagnostic callback changed profile mathematical result");
    }
}
