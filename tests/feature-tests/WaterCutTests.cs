using ImportEarthLocation.Core;
using ImportEarthLocation.Processing;
using Unity.Mathematics;

static class WaterCutTests
{
    static void Check(bool value,string message){if(!value)throw new Exception(message);}
    public static void Run()
    {
        const int n=33;const float scale=200;const double width=160;
        ushort H(double metres)=>(ushort)Math.Round(metres/scale*65535);
        var original=Enumerable.Repeat(H(100),n*n).ToArray();var shaped=original.ToArray();
        var owned=new byte[n*n];var channels=new byte[n*n];var controls=new HashSet<int>{16*n+10};
        for(var z=5;z<=27;z++)for(var x=9;x<=23;x++)
        {var i=z*n+x;shaped[i]=H(92);owned[i]=1;if(x>=15&&x<=17)channels[i]=1;}
        // A raised containment bank is owned, but is never part of this filter.
        shaped[16*n+25]=H(102);owned[16*n+25]=1;
        var before=shaped.ToArray();var mask=owned.ToArray();
        var log=new ImportLog();
        var count=WaterCutSmoothing.Apply(original,shaped,owned,channels,controls,scale,width,_=>98,
            Array.Empty<float3[]>(),log,default);
        Check(count>0&&shaped[16*n+9]>before[16*n+9],"A newly cut sharp bank was not softened");
        Check(original[16*n+8]-shaped[16*n+9]<original[16*n+8]-before[16*n+9],"Cut edge slope did not decrease");
        for(var i=0;i<shaped.Length;i++)
        {
            if(owned[i]==0||channels[i]!=0||controls.Contains(i)||before[i]>=original[i])
                Check(shaped[i]==before[i],"Bank smoothing touched uncut relief, a channel, control or levee");
            else Check(shaped[i]>=before[i]&&shaped[i]<=original[i]&&shaped[i]<=H(97.5),"Bank smoothing deepened a cut or filled above its wet ceiling");
        }
        Check(mask.SequenceEqual(owned),"Bank smoothing expanded edit ownership");
        // A mapped cliff may be finer than the DEM and still takes precedence.
        var mapped=before.ToArray();
        WaterCutSmoothing.Apply(original,mapped,owned,channels,controls,scale,width,_=>98,
            new[]{new[]{new float3(-35,0,-65),new float3(-35,0,65)}},new ImportLog(),default);
        Check(mapped[16*n+9]==before[16*n+9]&&mapped[16*n+23]>before[16*n+23],"Mapped cliff protection either failed or disabled unrelated bank smoothing");
        var cliffOriginal=original.ToArray();var cliffCut=before.ToArray();
        for(var z=0;z<n;z++)for(var x=0;x<=8;x++){cliffOriginal[z*n+x]=H(130);cliffCut[z*n+x]=H(130);}
        WaterCutSmoothing.Apply(cliffOriginal,cliffCut,owned,channels,controls,scale,width,_=>98,
            Array.Empty<float3[]>(),new ImportLog(),default);
        Check(cliffCut[16*n+9]==before[16*n+9],"Existing steep DEM relief was smoothed as an import cut");
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        try
        {
            WaterCutSmoothing.Apply(original,shaped,owned,channels,controls,scale,width,_=>98,Array.Empty<float3[]>(),new ImportLog(),cancel.Token);
            throw new Exception("Cut-bank smoothing ignored cancellation");
        }
        catch(OperationCanceledException){}
        Console.WriteLine("PASS: cut-bank feathering reduces new edge slopes while preserving untouched terrain, DEM and mapped cliffs, wet cores, controls, levees and edit ownership.");
    }
}
