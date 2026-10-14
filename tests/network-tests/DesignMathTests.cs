using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class DesignMathTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    static void Close(double3 a,double3 b,string message,double epsilon=1e-10)=>Check(math.distance(a,b)<=epsilon,message);
    public static void Run()
    {
        var c=NetworkDesignChecks.Hermite(new(0,10,0),new(300,25,0),new(1,.05,0),new(1,.05,0),300);
        Close(c.P1,new(100,15,0),"Hermite start handle reach");Close(c.P2,new(200,20,0),"Hermite end handle reach");
        Close(NetworkDesignChecks.Evaluate(c,0),c.P0,"Bezier start");Close(NetworkDesignChecks.Evaluate(c,1),c.P3,"Bezier end");
        Close(NetworkDesignChecks.Derivative(c,0),new(300,15,0),"Endpoint derivative is scaled by reference span");
        var split=NetworkDesignChecks.Split(c,.37);
        for(var i=0;i<=20;i++)
        {
            var t=i/20d;Close(NetworkDesignChecks.Evaluate(split.Left,t),new(300*.37*t,10+15*.37*t,0),"Left split analytic line");
            Close(NetworkDesignChecks.Evaluate(split.Right,t),new(300*(.37+.63*t),10+15*(.37+.63*t),0),"Right split analytic line");
        }
        Close(NetworkDesignChecks.Derivative(split.Left,1)/.37,NetworkDesignChecks.Derivative(split.Right,0)/.63,"Subdivision derivatives preserve physical tangent");
        var line=NetworkDesignChecks.Certify(c,1,.051);
        Check(line.Quality==DesignQuality.Certified&&Math.Abs(line.MaximumGrade-.05)<1e-12&&line.MaximumCurvature==0,"Straight grade certificate");
        var parabola=new Cubic3d(new(0,0,0),new(1/3d,0,0),new(2/3d,0,1/3d),new(1,0,1));
        for(var i=0;i<=20;i++)
        {
            var t=i/20d;Close(NetworkDesignChecks.Evaluate(parabola,t),new(t,0,t*t),"Degree-elevated parabola");
            Check(Math.Abs(NetworkDesignChecks.Curvature(parabola,t)-2/Math.Pow(1+4*t*t,1.5))<1e-10,"Analytical parabola curvature");
        }
        var certificate=NetworkDesignChecks.Certify(parabola,2.01,1);
        Check(certificate.Quality==DesignQuality.Certified&&certificate.MaximumCurvature>=2-1e-8&&certificate.MaximumCurvature<=2.01,"Conservative curvature certificate");
        foreach(var s in new[]{0d,220d})
        {var shift=NetworkDesignChecks.QuinticShift(4,s,220);Check(Math.Abs(shift.First)<1e-14&&Math.Abs(shift.Second)<1e-14,"Quintic C2 endpoints");}
        var middle=NetworkDesignChecks.QuinticShift(4,110,220);
        Check(Math.Abs(middle.Value-2)<1e-12&&Math.Abs(middle.First+1.875*4/220)<1e-12,"Quintic analytical midpoint/slope bound");
        var point=new Cubic3d(new(1,2,3),new(1,2,3),new(1,2,3),new(1,2,3));
        var degenerate=NetworkDesignChecks.Certify(point,1,1);
        Check(degenerate.Quality==DesignQuality.RequiresReview&&!degenerate.NonFinite&&degenerate.Leaves==1,"Finite degenerate quality is not nonfinite/native safety");
        Check(NetworkDesignChecks.Certify(c with{P1=new(double.NaN,0,0)},1,1).NonFinite,"Nonfinite native safety identified");
        var anchor=new double3(28672,100,28672);var shifted=c with{P0=c.P0+anchor,P1=c.P1+anchor,P2=c.P2+anchor,P3=c.P3+anchor};
        Close(NetworkDesignChecks.Evaluate(shifted,.23)-anchor,NetworkDesignChecks.Evaluate(c,.23),"Large-coordinate translation",1e-8);
        var leaves=new List<FootprintLeaf>();
        NetworkFootprintMath.BuildLeaves(new(new(-50,0,0),new(-50/3d,0,0),new(50/3d,0,0),new(50,0,0)),6.5,0,leaves,default);
        NetworkFootprintMath.BuildLeaves(new(new(0,5.25,-50),new(0,5.25,-50/3d),new(0,5.25,50/3d),new(0,5.25,50)),6.5,1,leaves,default);
        Check(leaves.Count==2&&leaves.All(l=>l.Certified),"Straight footprints must not need subdivision");
        var rectangle=new double2[16];var clip=new double2[16];var scratch=new double2[16];
        var count=NetworkFootprintMath.Rectangle(leaves[0],6.5,double2.zero,rectangle);
        NetworkFootprintMath.Rectangle(leaves[1],6.5,double2.zero,clip);
        count=NetworkFootprintMath.Intersect(rectangle,count,clip,4,scratch);
        Check(count==4&&Math.Abs(NetworkFootprintMath.TriangleArea(rectangle[0],rectangle[1],rectangle[2])*2-169)<1e-10,"Analytical13m cross-footprint area");
        // Both decks incline9%. Centre gap5.25m hides a corner gap4.08m.
        var minimum=rectangle.Min(p=>5.25+.09*p.y-.09*p.x);
        Check(Math.Abs(minimum-4.08)<1e-10&&minimum<5,"Centre-only crossing clearance must not certify the corners");
        var weights=new double[4];
        for(var i=0;i<10;i++)
        {
            NetworkFootprintMath.TriangleBlossom(.2,.4,.7,i,weights);
            Check(Math.Abs(weights.Sum()-1)<1e-12&&weights.All(w=>w>=0),"Correlated cubic Bernstein partition");
        }
        Console.WriteLine("PASS V-CROSS: correlated13m footprints, hidden inclined corner violation, cubic blossom partition.");
        var boxes=new List<FootprintLeaf>();
        for(var i=0;i<90;i++)
        {
            var lo=new double2((i%9)*7,(i/9)*6);var hi=lo+new double2(i%4==0?30:8,i%3==0?25:7);
            boxes.Add(new(i,i,0,1,default,default,default,default,1,0,1,lo,hi,true));
        }
        var hierarchy=new NetworkFootprintIndex(boxes,default);
        var indexed=hierarchy.CandidatePairs(default).ToHashSet();var exhaustive=new HashSet<(int,int)>();
        for(var i=0;i<boxes.Count;i++)for(var j=i+1;j<boxes.Count;j++)
            if(boxes[i].Min.x<=boxes[j].Max.x&&boxes[i].Max.x>=boxes[j].Min.x&&boxes[i].Min.y<=boxes[j].Max.y&&boxes[i].Max.y>=boxes[j].Min.y)exhaustive.Add((i,j));
        Check(indexed.SetEquals(exhaustive),"BVH contact coverage differs from independent exhaustive overlap");
        for(var i=0;i<boxes.Count;i++)boxes[i]=boxes[i] with{Min=default,Max=new double2(1,1)};
        var dense=new NetworkFootprintIndex(boxes,default).CandidatePairs(default).ToArray();
        Check(dense.Length==90*89/2&&dense.Distinct().Count()==dense.Length,"Dense contact coverage truncated/duplicated");
        Console.WriteLine("PASS V-CROSS index: exhaustive mixed/long-box equivalence and4005 unique dense pairs without caps.");
        var fit=NetworkDesignChecks.FitHandles(new(0,0,0),new(300,0,0),new(1,0),new(1,0),
            new[]{new double3(0,0,0),new double3(60,0,0),new double3(180,0,0),new double3(300,0,0)},
            new[]{0d,.2,.6,1},new[]{30d,90,120,60},1e-6);
        Close(fit.P1,new(100,0,0),"Independent LS straight handle",1e-8);Close(fit.P2,new(200,0,0),"Independent LS straight handle",1e-8);
        var boxed=NetworkDesignChecks.FitHandles(new(0,0,0),new(100,0,0),new(1,0),new(1,0),
            new[]{new double3(0,0,0),new double3(1000,0,0),new double3(100,0,0)},new[]{0d,.5,1},new[]{1d,1,1},1e-6);
        Check(boxed.P1.x>=5&&boxed.P1.x<=45&&100-boxed.P2.x>=5&&100-boxed.P2.x<=45,"Box-constrained fitter escaped physical reach");
        Console.WriteLine("PASS V-FIT handles: analytical straight optimum and bounded extreme observation.");
        var gamma=new double[6];var elevated=new double[10];var product=new double[10];
        var ta=new double2(.1,.2);var tb=new double2(.8,.1);var tc=new double2(.3,.9);
        NetworkTerrainPolynomial.Quadratic(10,13,15,24,ta,tb,tc,gamma);
        var affine=new double3(.2,.6,.9);
        for(var i=0;i<10;i++){elevated[i]=NetworkTerrainPolynomial.Elevate(gamma,i);product[i]=NetworkTerrainPolynomial.AffineTimesQuadratic(affine,gamma,i);}
        foreach(var bary in new[]{new double3(1,0,0),new double3(0,1,0),new double3(0,0,1),new double3(.2,.3,.5),new double3(.7,.2,.1)})
        {
            var cell=ta*bary.x+tb*bary.y+tc*bary.z;var exact=10+3*cell.x+5*cell.y+6*cell.x*cell.y;
            Check(Math.Abs(NetworkTerrainPolynomial.EvaluateCubic(elevated,bary)-exact)<1e-11,"Bilinear terrain degree elevation is not exact");
            Check(Math.Abs(NetworkTerrainPolynomial.EvaluateCubic(product,bary)-math.dot(affine,bary)*exact)<1e-11,"Portal terrain blend multiplication is not exact");
        }
        foreach(var distance in new[]{0d,32,80,128,160})
        {var u=distance/160;Check(Math.Abs(NetworkTerrainPolynomial.CoverBlossom(distance,distance,distance)-15*(3*u*u-2*u*u*u))<1e-12,"Independent smoothstep cover polynomial");}
        Console.WriteLine("PASS V-TUNNEL polynomial: exact bilinear elevation, affine portal blend, analytical15m cover ramp.");
        var adjacency=new[]{new[]{new ProfileGraphArc(1,5),new ProfileGraphArc(2,20)},
            new[]{new ProfileGraphArc(0,5),new ProfileGraphArc(2,7)},
            new[]{new ProfileGraphArc(0,20),new ProfileGraphArc(1,7),new ProfileGraphArc(3,2)},
            new[]{new ProfileGraphArc(2,2)},Array.Empty<ProfileGraphArc>()};
        var graph=new NetworkProfileGraph(adjacency);var distances=graph.Distances(new[]{0,3},double.PositiveInfinity,default);
        Check(distances.SequenceEqual(new[]{0d,5,2,0,double.PositiveInfinity}),"Independent multi-source portal graph shortest paths");
        var cutoff=graph.Distances(new[]{0},6,default);
        Check(cutoff[1]==5&&double.IsPositiveInfinity(cutoff[2]),"Witness graph budget changed reachable set");
        // Frozen graph must not change when the caller edits its source arrays.
        adjacency[0][0]=new ProfileGraphArc(1,99);Check(graph.Distances(new[]{0},6,default)[1]==5,"Frozen adjacency alias mutated");
        Console.WriteLine("PASS V-TUNNEL/V-PROFILE graph: multi-source distances, bounded search and immutable adjacency.");
        Console.WriteLine("PASS V-FIT/V-JOIN: analytical Bezier/Hermite, subdivision, continuous grade/curvature, quintic C2, precision and degenerate quality/safety checks.");
    }
}
