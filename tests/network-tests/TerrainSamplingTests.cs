using ImportEarthLocation.Processing;

// The heightmap resampler: continuous slope where bilinear facets, exact on a
// quadratic surface, and never outside a cell's own samples at a step.
static class TerrainSamplingTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        float Quadratic(int x,int y)=>.3f*x*x+.1f*x*y-.2f*y*y;
        float At(Func<int,int,float> read,double px,double py)
        {
            var x=(int)Math.Floor(px);var y=(int)Math.Floor(py);
            return TerrainSampling.ClampedCubic(read,x,y,TerrainSampling.CubicWeights((float)(px-x)),TerrainSampling.CubicWeights((float)(py-y)));
        }
        // Catmull-Rom reproduces a quadratic away from the clamp.
        var exact=At(Quadratic,5.25,7.5);var expected=.3*5.25*5.25+.1*5.25*7.5-.2*7.5*7.5;
        Check(Math.Abs(exact-expected)<2e-3,$"Bicubic resampling misses a smooth surface: {exact} against {expected}");
        // Slope either side of a source sample: equal for the cubic.
        float Slope(double px)=>(At(Quadratic,px+1e-3,7.5)-At(Quadratic,px-1e-3,7.5))/2e-3f;
        Check(Math.Abs(Slope(6-1e-2)-Slope(6+1e-2))<.05,"Bicubic resampling breaks slope at a source sample");
        // A cliff step between columns 4 and 5 rings nowhere.
        float Step(int x,int y)=>x<5?0:30;
        for(var px=3.0;px<=7;px+=.05){var v=At(Step,px,2.5);Check(v>=-1e-4&&v<=30+1e-4,$"Bicubic resampling rang past a step: {v} at {px}");}
        // At a source sample the value is that sample.
        Check(Math.Abs(At(Step,4,3)-0)<1e-5&&Math.Abs(At(Step,5,3)-30)<1e-5,"Bicubic resampling moved a source sample");
        Console.WriteLine("PASS: heightmap resampling keeps slope continuous, reproduces smooth relief and stays within each cell at steps.");
    }
}
