using ImportEarthLocation.Processing;

const int size = 128;
int At(int x, int y) => y * size + x;
float[] Raster(Func<int,int,float> height) => Enumerable.Range(0, size * size).Select(i => height(i % size, i / size)).ToArray();
void Check(bool value, string message) { if (!value) throw new Exception(message); }
TerrainConditioner.Result Run(float[] source, Func<int,bool>? guard = null, int tileSize = 256) => TerrainConditioner.Condition(source, size, size, 30, default, guard, tileSize);
void Reject(Action action, string message) { try { action(); } catch (InvalidOperationException) { return; } throw new Exception(message); }

foreach(var bounds in new[]{(-11641.5f,643.8f),(-12000f,10000f),(-20f,3f),(0f,0f),(100f,900f)})
{
    var encoding=TerrainConditioner.EncodingFor(bounds.Item1,bounds.Item2);
    Check(encoding.BaseElevation<=bounds.Item1-128&&encoding.BaseElevation+encoding.Scale>=bounds.Item2,
        "Validated DEM bounds overflow terrain encoding");
    Check(encoding.SeaLevel==-encoding.BaseElevation,"Sea level lost consistent zero-elevation datum");
    foreach(var value in new[]{bounds.Item1,bounds.Item2,0f})
    {
        var encoded=Math.Round((value-encoding.BaseElevation)/encoding.Scale*65535);
        var decoded=encoded/65535*encoding.Scale+encoding.BaseElevation;
        Check(encoded>=0&&encoded<=65535&&Math.Abs(decoded-value)<=encoding.Scale/65535*.501,
            "DEM encoding clips or loses more than half a quantization step");
    }
}
foreach(var bounds in new[]{(float.NaN,10f),(-12001f,10f),(0f,10001f),(10f,0f)})
    Reject(()=>TerrainConditioner.EncodingFor(bounds.Item1,bounds.Item2),"Invalid DEM bounds accepted");
Console.WriteLine("PASS: Santa Catalina 12,285.3m span and full accepted elevation range encode without clipping; invalid samples remain rejected.");

var slope = Raster((x,y) => 100 + x * .2f + y * .3f);
var reference = (float[])slope.Clone();
Check(Run(slope).Elevations.SequenceEqual(slope), "Planar slope was blurred");
Check(reference.SequenceEqual(slope), "Original DEM was mutated");
var progress=new List<string>();
TerrainConditioner.Condition(reference,size,size,30,default,progress:progress.Add);
Check(progress.Any(s=>s.Contains("source-validation"))&&progress.Any(s=>s.Contains("tile-seam"))&&progress.Any(s=>s.Contains("noise/spike")),"DEM calculation progress stages missing");
slope[At(60,60)] += 30; slope[At(90,90)] -= 20;
var repaired = Run(slope);
Check(repaired.Metrics.Spikes == 1 && repaired.Metrics.Pits == 1, "Isolated artifacts not detected");
Check(Math.Abs(repaired.Elevations[At(60,60)] - reference[At(60,60)]) < .001, "Slope not preserved by spike repair");
Check(slope[At(60,60)] == reference[At(60,60)] + 30, "Artifact repair modified source");
Check(Run(slope, i => i == At(60,60)).Elevations[At(60,60)] == slope[At(60,60)], "Protected summit/quarry changed");
var coast = Raster((x,y) => x < 64 ? -20 : 3);
coast[At(20,20)] = 20; // Tiny positive island surrounded by sea must not become a 'spike'.
Check(Run(coast).Elevations.SequenceEqual(coast), "Coast or isolated island erased");
var cliff = Raster((x,y) => x < 64 ? 100 : 350);
Check(Run(cliff, tileSize:64).Elevations.SequenceEqual(cliff), "Tile-aligned real cliff rejected or smoothed");
var ridge = Raster((x,y) => 100 + Math.Max(0, 30 - Math.Abs(x - 64) * 3));
Check(Run(ridge).Elevations.SequenceEqual(ridge), "Coherent ridge rounded");
var basin = Raster((x,y) => 100 - Math.Max(0, 25 - Math.Abs(x-64) - Math.Abs(y-64)));
Check(Run(basin).Elevations.SequenceEqual(basin), "Coherent valley/quarry basin changed");
var holes = (float[])reference.Clone(); holes[At(40,40)] = float.NaN; holes[At(41,40)] = float.NaN;
Check(Math.Abs(Run(holes).Elevations[At(40,40)] - reference[At(40,40)]) < .001, "Bounded NoData repair flattened slope");
Reject(() => Run(holes, i => i == At(40,40)), "Protected shoreline NoData invented");
var largeHole = (float[])reference.Clone(); for(var x=30;x<35;x++) largeHole[At(x,30)] = float.NaN;
Reject(() => Run(largeHole), "Large NoData hole accepted");
Reject(() => Run(Raster((x,y) => float.NaN)), "Ocean NoData assumed zero without mask");
var invalid = (float[])reference.Clone(); invalid[0] = float.PositiveInfinity;
Reject(() => Run(invalid), "Infinite elevation accepted"); invalid[0] = 15000;
Reject(() => Run(invalid), "Impossible elevation accepted");
Reject(() => TerrainConditioner.Condition(reference, size, size, 30, default, tileSize:0), "Invalid tile size can loop forever");
var noise = Raster((x,y) => 100); noise[At(64,64)] += .8f;
Check(Run(noise).Elevations[At(64,64)] < noise[At(64,64)] && Run(noise).Elevations[At(64,64)] > 100.6f, "Noise smoothing not light/bounded");
var budget = Raster((x,y) => 100); for(var y=5;y<size-5;y+=5) for(var x=5;x<size-5;x+=5) budget[At(x,y)] = 140;
var reverted = Run(budget);
Check(reverted.Elevations.SequenceEqual(budget) && reverted.Metrics.ValidationNote.Contains("reverted"), "Unsafe optional filter did not revert cleanly");
using(var cancellation = new CancellationTokenSource())
{ cancellation.Cancel(); try { TerrainConditioner.Condition(reference,size,size,30,cancellation.Token); throw new Exception("Cancellation ignored"); } catch(OperationCanceledException) {} }
var original = Raster((x,y) => 100); var working = (float[])original.Clone(); var corridor = new bool[original.Length]; var rim = new bool[original.Length];
corridor[100] = corridor[101] = true;
TerrainConditioner.ApplyHydrologyCorrections(original, working, corridor, rim, new[]{new TerrainConditioner.HydrologyCorrection(100,98,"verified stream")}, default);
Check(original[100] == 100 && working[100] == 98, "Hydrology did not preserve immutable reference");
var before = (float[])working.Clone();
Reject(() => TerrainConditioner.ApplyHydrologyCorrections(original,working,corridor,rim,new[]{new TerrainConditioner.HydrologyCorrection(100,97,"valid"),new TerrainConditioner.HydrologyCorrection(101,96,"too deep")},default), "Canyon carving accepted");
Check(before.SequenceEqual(working), "Rejected hydrology partially modified working buffer");
rim[101] = true;
Reject(() => TerrainConditioner.ApplyHydrologyCorrections(original,working,corridor,rim,new[]{new TerrainConditioner.HydrologyCorrection(101,99,"rim")},default), "Protected lake rim lowered");
Console.WriteLine("PASS: immutable source, slopes, bounded spikes/pits/noise, coasts/islands, cliff seams, ridges/basins, NoData budgets/protection, invalid DEM rejection, filter rollback, cancellation, atomic <=3m hydrology/rim constraints.");
