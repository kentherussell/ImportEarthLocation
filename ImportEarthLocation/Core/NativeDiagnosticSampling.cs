using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

public static class NativeDiagnosticSampling
{
    // Water-fill small strata first, then redistribute every unused slot.
    // This affects diagnostic coverage only, never which networks are placed.
    public static int[] Quotas(IReadOnlyList<int> counts,int maximum)
    {
        var output=new int[counts.Count];long total=0;
        foreach(var count in counts){if(count<0)throw new ArgumentOutOfRangeException(nameof(counts));total+=count;}
        var remaining=(int)Math.Min(Math.Max(0,maximum),total);
        while(remaining>0)
        {
            var active=0;for(var i=0;i<counts.Count;i++)if(output[i]<counts[i])active++;
            if(active==0)break;
            var share=Math.Max(1,remaining/active);
            for(var i=0;i<counts.Count&&remaining>0;i++)
            {var take=Math.Min(Math.Min(share,remaining),counts[i]-output[i]);output[i]+=take;remaining-=take;}
        }
        return output;
    }
}
