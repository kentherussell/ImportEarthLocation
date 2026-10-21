using ImportEarthLocation.Processing;

static class ProfileStarCoordinateTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        var rows=new SparseProfileRows(3);
        rows.Append(new[]{0},new[]{1d},1,3,2,ProfileRowKind.EqualityCost);
        rows.Append(new[]{1},new[]{1d},1,1,3,ProfileRowKind.EqualityCost);
        rows.Append(new[]{2},new[]{1d},1,7,1,ProfileRowKind.EqualityCost);
        rows.Append(new[]{0,1,2},new[]{2d,-1,-1},3,-4,.5,ProfileRowKind.UpperHingeCost);
        rows.Append(Array.Empty<int>(),Array.Empty<double>(),0,-2,5,ProfileRowKind.UpperHingeCost);rows.Freeze();
        var star=new ProfileStar(0,new[]{1,2},new[]{-1d,2},new[]{0,0});
        var coordinates=ProfileStarCoordinates.Create(rows,new double[3],new(3,10,new[]{star}),default);
        Check(coordinates!=null&&coordinates.Rows.Variables==2,"Free star coordinates did not eliminate the exact shared-grade relation");
        var c=coordinates!;var random=new Random(72819);
        var vertices=new[]{new[]{0d,0},new[]{10d/3,20d/3},new[]{10d,0},new[]{20d/3,-20d/3}};
        double Cost(double[] p,double[] trial,double[] d)=>Enumerable.Range(0,p.Length).Sum(i=>d[i]*(p[i]-trial[i])*(p[i]-trial[i]));
        for(var iteration=0;iteration<250;iteration++)
        {
            var z=new[]{random.NextDouble()*50-20,random.NextDouble()*40-20};var x=c.Expand(z);
            var gx=new double[3];var gz=new double[2];var a=rows.Evaluate(x,gx,default);var b=c.Rows.Evaluate(z,gz,default);
            Check(Math.Abs(a-b)<=1e-10*Math.Max(1,Math.Abs(a)),"Star substitution changed hinge/equality/constant objective");
            var mapped=new double[2];for(var i=0;i<3;i++){var binding=c.Bindings[i];mapped[binding.Height]+=gx[i];if(binding.Relative>=0)mapped[binding.Relative]+=binding.Beta*gx[i];}
            Check(mapped.Zip(gz).All(p=>Math.Abs(p.First-p.Second)<=1e-9*Math.Max(1,Math.Abs(p.First))),"Star substitution gradient is not the original gradient under the chain rule");
            var d=new[]{Math.Pow(10,random.NextDouble()*8-4),Math.Pow(10,random.NextDouble()*8-4)};
            var projected=(double[])z.Clone();Check(c.Projection.Project(projected,d),"Finite weighted star projection unavailable");
            var expanded=c.Expand(projected);Check(expanded.All(v=>v>=-1e-9&&v<=10+1e-9),"Relative coordinates escaped an original height box");
            // Independent closest-point enumeration over the four polygon edges.
            var expected=x.All(v=>v>=0&&v<=10)?(double[])z.Clone():vertices[0];var best=Cost(expected,z,d);
            for(var edge=0;edge<4;edge++)
            {
                var p=vertices[edge];var q=vertices[(edge+1)%4];var delta=new[]{q[0]-p[0],q[1]-p[1]};
                var t=Math.Clamp((d[0]*(z[0]-p[0])*delta[0]+d[1]*(z[1]-p[1])*delta[1])/(d[0]*delta[0]*delta[0]+d[1]*delta[1]*delta[1]),0,1);
                var candidate=new[]{p[0]+t*delta[0],p[1]+t*delta[1]};var cost=Cost(candidate,z,d);
                if(cost<best){best=cost;expected=candidate;}
            }
            Check(Cost(projected,z,d)<=best+1e-8*Math.Max(1,best)&&projected.Zip(expected).All(p=>Math.Abs(p.First-p.Second)<1e-5),
                "Weighted star projection disagrees with independent polygon-edge enumeration");
            var gradient=new[]{random.NextDouble()*20-10,random.NextDouble()*20-10};
            var support=vertices.Min(p=>p[0]*gradient[0]+p[1]*gradient[1]);
            Check(c.Projection.TryLinearMinimum(gradient,out var lower)&&lower<=support+1e-10&&support-lower<1e-7,
                "Relative-coordinate linear support is not a valid tight lower bound");
        }
        var stiff=new SparseProfileRows(3);
        stiff.Append(new[]{0},new[]{1d},1,1,1,ProfileRowKind.EqualityCost);
        stiff.Append(new[]{0,1,2},new[]{-3d,2,1},3,0,1e12,ProfileRowKind.EqualityCost);
        stiff.Append(new[]{1,2},new[]{-1d,1},2,0,1,ProfileRowKind.EqualityCost);
        for(var i=0;i<3;i++)stiff.Append(new[]{i},new[]{1d},1,0,1e-8,ProfileRowKind.EqualityCost);stiff.Freeze();
        var result=NetworkProfileSolve.SolveComponent(stiff,new double[3],new(3,10,new[]{star}));
        var m=-1e-8/(9+5e-8);var h=1/(1+3e-8+1e-8*m);m*=h;
        Check(result.Report.Outcome==ProfileSolveOutcome.Converged&&result.Report.AcceptedIterations<=2000&&
            Math.Abs(result.Heights[0]-h)<.0005&&Math.Abs(result.Heights[1]-(h-m))<.0005&&Math.Abs(result.Heights[2]-(h+2*m))<.0005,
            $"Stiff hard-star problem did not reach its independent optimum: {result.Report}");
        Check(result.Report.OriginalVariables==3&&result.Report.OptimizerVariables==2,"Actual solver coordinate dimensions missing from evidence");
        Check(ProfileStarCoordinates.Create(rows,new double[3],new(3,10,new[]{new ProfileStar(0,new[]{1,2},new[]{-1d,2},new[]{0,0},fixedHeight:4)}),default)==null,
            "Verified fixed native context was silently reparameterized");
        var constantRows=new SparseProfileRows(3);constantRows.Append(new[]{0,1,2},new[]{-3d,2,1},3,1,2,ProfileRowKind.EqualityCost);constantRows.Freeze();
        var constantResult=NetworkProfileSolve.SolveComponent(constantRows,new[]{5d,5,5},new(3,10,new[]{star}));
        Check(constantResult.Report.Outcome==ProfileSolveOutcome.Converged&&Math.Abs(constantResult.Report.Objective-1)<1e-10&&constantRows.Count==1,
            "A constant-on-hard-set contradiction became numerical fallback or disappeared from evidence");
        var noDependence=new SparseProfileRows(3);noDependence.Append(Array.Empty<int>(),Array.Empty<double>(),0,-2,2,ProfileRowKind.UpperHingeCost);noDependence.Freeze();
        var noDependenceResult=NetworkProfileSolve.SolveComponent(noDependence,new[]{5d,5,5},new(3,10,new[]{star}));
        Check(noDependenceResult.Report.Outcome==ProfileSolveOutcome.Converged&&noDependenceResult.Report.Objective==4,
            "Zero-curvature source certificate rejected a feasible constant objective");
        var wideRows=new SparseProfileRows(24);var wideStars=Enumerable.Range(0,8).Select(i=>new ProfileStar(i*3,new[]{i*3+1,i*3+2},new[]{-1d,2},new[]{0,0})).ToArray();
        wideRows.Append(Enumerable.Range(0,8).Select(i=>i*3+1).ToArray(),Enumerable.Repeat(1d,8).ToArray(),8,1,1,ProfileRowKind.EqualityCost);wideRows.Freeze();
        Check(ProfileStarCoordinates.Create(wideRows,new double[24],new(24,10,wideStars),default)==null&&wideRows.Row(0).Count==8,
            "Oversized transformed row was truncated instead of retaining the original problem");
        var multiRows=new SparseProfileRows(5);for(var i=0;i<5;i++)multiRows.Append(new[]{i},new[]{1d},1,0,1,ProfileRowKind.EqualityCost);multiRows.Freeze();
        var multi=ProfileStarCoordinates.Create(multiRows,new double[5],new(5,10,new[]{new ProfileStar(0,new[]{1,2,3,4},new[]{-1d,2,-3,1},new[]{0,0,1,1})}),default)!;
        for(var iteration=0;iteration<50;iteration++)
        {
            var trial=new[]{random.NextDouble()*30-10,random.NextDouble()*30-15,random.NextDouble()*30-15};
            var weights=new[]{.2+random.NextDouble()*10,.2+random.NextDouble()*10,.2+random.NextDouble()*10};
            double Q(double h,int group)
            {var p=group==0?1:1d/3;var n=group==0?.5:1;return Math.Clamp(trial[group+1],Math.Max(-h/p,(h-10)/n),Math.Min((10-h)/p,h/n));}
            double Objective(double h)=>weights[0]*(h-trial[0])*(h-trial[0])+Enumerable.Range(0,2).Sum(g=>weights[g+1]*(Q(h,g)-trial[g+1])*(Q(h,g)-trial[g+1]));
            // Independent scalar convex minimization, not the production
            // breakpoint enumeration or its active affine formulas.
            var lo=0d;var hi=10d;for(var k=0;k<120;k++){var left=(2*lo+hi)/3;var right=(lo+2*hi)/3;if(Objective(left)<Objective(right))hi=right;else lo=left;}
            var candidate=(double[])trial.Clone();Check(multi.Projection.Project(candidate,weights),"Multi-grade-group projection unavailable");
            Check(Cost(candidate,trial,weights)<=Objective((lo+hi)/2)+1e-8*Math.Max(1,Objective((lo+hi)/2))&&Math.Abs(candidate[0]-(lo+hi)/2)<1e-5,
                "Multi-group weighted projection disagrees with independent convex minimization");
        }
        var metricRows=new SparseProfileRows(2);
        metricRows.Append(new[]{0},new[]{1d},1,3,1,ProfileRowKind.EqualityCost);
        metricRows.Append(new[]{1},new[]{1d},1,0,1,ProfileRowKind.EqualityCost);
        metricRows.Append(new[]{1},new[]{1000d},1,1000,1,ProfileRowKind.UpperHingeCost);metricRows.Freeze();
        var metric=metricRows.Conditioning(default);
        Check(metric.D[1]==1000001&&metric.L>=1000001d/metric.D[1],"Inactive hinge escaped the all-row metric or global bound");
        Check(Math.Abs(metricRows.Evaluate(new[]{0d,2},null,default)-500006.5)<1e-8,"Inactive metric policy dropped a hinge from the actual objective");
        for(var k=0;k<50;k++)
        {var a=random.NextDouble()*2-1;var b=random.NextDouble()*2-1;Check(a*a+1000001*b*b<=metric.L*(metric.D[0]*a*a+metric.D[1]*b*b)*(1+1e-12),"Active metric global Lipschitz bound is not conservative");}
        Console.WriteLine($"PASS: exact star objective/gradient, independent weighted polygon projection/support and stiff constrained optimum; iterations={result.Report.AcceptedIterations}.");
    }
}
