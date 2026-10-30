using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Per-class visual design values from research report 64 §7. Lengths and radii
// are metres; grades are fractions; K is metres of road per 1% grade change.
// Starting values to tune against screenshots, never against float residuals.
public readonly record struct VisualCompilerClass(double Epsilon,double Deviation,double RadiusTarget,double RadiusFloor,double Wavelength,double K,
    double PlateauGrade,double PlateauLength,double MinimumSegment,double ClusterLength,double PairSeparation,double MergeTolerance);

public static class VisualCompilerTable
{
    // Inside a bore a vertical curve is out of sight, so tunnels allow twice the
    // class grade-change rate (half its K) to reach cover; portals keep the
    // open-road rate.
    public const double TunnelKShare=.5;
    public static VisualCompilerClass For(PreparedSegment s)
    {
        // Wavelengths were raised about 2.5× from report 64 after replay: at 400 m a
        // motorway still followed kilometre-scale hills with vertical curves far
        // tighter than its K. Longer smoothing trades them for cut and fill.
        if(s.Kind==NetworkKind.Rail)return new(3,4,800,s.NativeCenterlineCurvatureLimit is double k and >0?1/k:300,1200,60,.015,0,60,0,0,.3);
        // A pinned hump of height H peaks at curvature ≈ H/(2ℓ²), ℓ = Λ/2π. A ramp
        // bridge rising 7.5 m within K = 15 needs ℓ ≥ 75 m, so Λ ≥ about 470 m.
        if(s.ProtectedLink)return new(2,4,150,50,600,15,.03,20,30,0,0,.5);
        var c=Road(s.Classification);
        // One carriageway of a divided road sits 10–30 m from its partner: a
        // metre-scale tolerance keeps the pair apart while fillets still smooth it.
        return s.OneWay&&c.PairSeparation>0?c with{Epsilon=1,Deviation=1.5}:c;
    }

    private static VisualCompilerClass Road(string classification)
    {
        return classification switch
        {
            "motorway"=>new(4,8,600,250,1000,40,.025,40,60,0,60,.3),
            "trunk"=>new(3,6,400,150,800,30,.03,30,50,25,50,.3),
            "primary"=>new(2.5,4,200,60,500,15,.03,25,40,20,40,.5),
            "secondary"=>new(2,3,120,40,350,10,.04,20,30,15,30,.5),
            "tertiary" or "unclassified"=>new(1.5,2.5,80,25,250,8,.04,15,25,12,0,.5),
            _=>new(1,2,40,12,150,5,.05,12,15,8,0,.5)
        };
    }
}
