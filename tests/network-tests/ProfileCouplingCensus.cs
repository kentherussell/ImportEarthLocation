using ImportEarthLocation.Processing;

// Offline all-row local blocks; this is not a global condition-number estimate.
static class ProfileCouplingCensus
{
    public sealed record Entry(int Height,int Controls,double Hh,double Hq,double Qq,double ActiveHh,double ActiveHq,double ActiveQq,
        long CoefficientVisits,long ActiveCoefficientVisits)
    {
        public double Correlation=>CorrelationOf(Hh,Hq,Qq);
        public double ActiveCorrelation=>CorrelationOf(ActiveHh,ActiveHq,ActiveQq);
        static double CorrelationOf(double h,double cross,double q)=>h>0&&q>0?Math.Abs(cross/Math.Sqrt(h)/Math.Sqrt(q)):0;
    }
    public sealed record Census(long Rows,long Terms,int Stars,int EligibleStars,bool ExistingTransformAvailable,Entry[] Entries)
    {
        public int HighCouplingStars=>Entries.Count(e=>e.Correlation>.8);
        public long HighCouplingCoefficientVisits=>Entries.Where(e=>e.Correlation>.8).Sum(e=>e.CoefficientVisits);
        public long EligibleCoefficientVisits=>Entries.Sum(e=>e.CoefficientVisits);
        public int ActiveHighCouplingStars=>Entries.Count(e=>e.ActiveCorrelation>.8);
        public long ActiveHighCouplingCoefficientVisits=>Entries.Where(e=>e.ActiveCorrelation>.8).Sum(e=>e.ActiveCoefficientVisits);
        public int NonfiniteLocalBlocks=>Entries.Count(e=>!double.IsFinite(e.Hh)||!double.IsFinite(e.Hq)||!double.IsFinite(e.Qq)||
            !double.IsFinite(e.ActiveHh)||!double.IsFinite(e.ActiveHq)||!double.IsFinite(e.ActiveQq));
        public object[] Histogram=>new[]{0d,.2,.4,.6,.8,1.000000000001}.Skip(1).Select((upper,i)=>
        {
            var lower=new[]{0d,.2,.4,.6,.8}[i];var all=Entries.Where(e=>e.Correlation>=lower&&e.Correlation<upper).ToArray();
            var active=Entries.Where(e=>e.ActiveCorrelation>=lower&&e.ActiveCorrelation<upper).ToArray();
            return (object)new{lowerInclusive=lower,upperExclusive=upper,stars=all.Length,coefficientVisits=all.Sum(e=>e.CoefficientVisits),
                activeStars=active.Length,activeCoefficientVisits=active.Sum(e=>e.ActiveCoefficientVisits)};
        }).ToArray();
        public string Scope=>"All-row and active-at-prior single-relative-star local Hessian blocks; other-star couplings omitted; not a speed or global conditioning certificate";
    }
    public static Census Analyze(SparseProfileRows rows,double[] prior,IReadOnlyList<ProfileStar> stars,CancellationToken token=default)
    {
        var eligible=stars.Where(s=>!s.FixedHeight.HasValue&&s.FixedSlopes.All(v=>!v.HasValue)&&s.GroupCount==1&&s.Controls.Length>0).ToArray();
        var owner=Enumerable.Repeat(-1,rows.Variables).ToArray();var beta=new double[rows.Variables];
        for(var s=0;s<eligible.Length;s++)
        {
            var star=eligible[s];var scale=star.Alpha.Max(Math.Abs);owner[star.Height]=s;
            for(var j=0;j<star.Controls.Length;j++){owner[star.Controls[j]]=s;beta[star.Controls[j]]=star.Alpha[j]/scale;}
        }
        var h=new double[eligible.Length];var cross=new double[h.Length];var q=new double[h.Length];
        var ah=new double[h.Length];var ac=new double[h.Length];var aq=new double[h.Length];var visits=new long[h.Length];var activeVisits=new long[h.Length];
        var ids=new int[8];var hc=new double[8];var qc=new double[8];var counts=new int[8];
        for(long r=0;r<rows.Count;r++)
        {
            if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(r);var used=0;var residual=-row.Target;
            for(var j=0;j<row.Count;j++)
            {
                var index=rows.Index(row.Offset+j);var value=rows.Coefficient(row.Offset+j);residual+=value*prior[index];
                var s=owner[index];if(s<0)continue;var slot=0;while(slot<used&&ids[slot]!=s)slot++;
                if(slot==used){ids[used]=s;hc[used]=qc[used]=0;counts[used++]=0;}
                hc[slot]+=value;qc[slot]+=value*beta[index];counts[slot]++;
            }
            var active=row.Kind==ProfileRowKind.EqualityCost||residual>0;
            for(var j=0;j<used;j++)
            {
                var s=ids[j];var hh=row.Weight*hc[j]*hc[j];var hq=row.Weight*hc[j]*qc[j];var qq=row.Weight*qc[j]*qc[j];
                h[s]+=hh;cross[s]+=hq;q[s]+=qq;visits[s]+=counts[j];
                if(active){ah[s]+=hh;ac[s]+=hq;aq[s]+=qq;activeVisits[s]+=counts[j];}
            }
        }
        var entries=eligible.Select((s,i)=>new Entry(s.Height,s.Controls.Length,h[i],cross[i],q[i],ah[i],ac[i],aq[i],visits[i],activeVisits[i])).ToArray();
        return new(rows.Count,rows.Terms,stars.Count,eligible.Length,stars.All(s=>!s.FixedHeight.HasValue&&s.FixedSlopes.All(v=>!v.HasValue)),entries);
    }
    public static void Test()
    {
        var rows=new SparseProfileRows(3);
        rows.Append(new[]{1},new[]{1d},1,0,1,ProfileRowKind.EqualityCost);
        rows.Append(new[]{2},new[]{1d},1,10,.01,ProfileRowKind.UpperHingeCost);rows.Freeze();
        var star=new ProfileStar(0,new[]{1,2},new[]{1d,-1d},new[]{0,0});
        var census=Analyze(rows,new[]{0d,0d,0d},new[]{star});var e=census.Entries.Single();
        if(Math.Abs(e.Hh-1.01)>1e-12||Math.Abs(e.Hq-.99)>1e-12||Math.Abs(e.Qq-1.01)>1e-12||
            e.ActiveHh!=1||e.ActiveHq!=1||e.ActiveQq!=1||census.HighCouplingStars!=1||e.CoefficientVisits!=2||e.ActiveCoefficientVisits!=1)
            throw new Exception("Single-star coupling census lost analytical all-row/active local blocks.");
        using var cancelled=new CancellationTokenSource();cancelled.Cancel();var rejected=false;
        try{Analyze(rows,new[]{0d,0d,0d},new[]{star},cancelled.Token);}catch(OperationCanceledException){rejected=true;}
        if(!rejected)throw new Exception("Coupling census ignored cancellation.");
    }
}
