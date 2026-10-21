using System.Diagnostics;
using ImportEarthLocation.Processing;

// Offline and invocation-owned. Chunk-local groups bound scratch independently
// of city size; targets/weights do not participate in coefficient identity.
static class ProfileRowCensus
{
    public sealed record Report(long Rows,long Terms,int Variables,long VariableRows,long ConstantRows,
        long[][] RowsByKindAndArity,long[] GroupHistogram,long EligibleRows,long EligibleTerms,
        long DotProductsAvoided,long EqualityGroups,long HingeGroups,long ScratchBytes,double Seconds);

    public static Report Inspect(SparseProfileRows rows,CancellationToken token=default)
    {
        if(rows.MeasurementOnly)throw new ArgumentException("Census requires retained rows.");
        var clock=Stopwatch.StartNew();var byKind=new[]{new long[9],new long[9]};var histogram=new long[5];
        var order=new int[SparseProfileRows.ChunkSize];long variables=0,constants=0,eligible=0,terms=0,avoided=0,eq=0,hinge=0;
        for(long begin=0;begin<rows.Count;begin+=order.Length)
        {
            token.ThrowIfCancellationRequested();var start=begin;var count=(int)Math.Min(order.Length,rows.Count-begin);var used=0;
            int Compare(int a,int b)
            {
                var x=rows.Row(start+a);var y=rows.Row(start+b);var c=x.Kind.CompareTo(y.Kind);
                if(c!=0)return c;c=x.Count.CompareTo(y.Count);if(c!=0)return c;
                for(var j=0;j<x.Count;j++)
                {
                    c=rows.Index(x.Offset+j).CompareTo(rows.Index(y.Offset+j));if(c!=0)return c;
                    var aa=rows.Coefficient(x.Offset+j);var bb=rows.Coefficient(y.Offset+j);
                    // All stored coefficients are finite. Numeric equality also
                    // agrees with the existing canonicalization of signed zero.
                    c=aa.CompareTo(bb);if(c!=0)return c;
                }
                return 0;
            }
            for(var r=0;r<count;r++)
            {
                if((r&1023)==0)token.ThrowIfCancellationRequested();var row=rows.Row(begin+r);
                byKind[(int)row.Kind][row.Count]++;
                if(row.Count==0){constants++;continue;}variables++;order[used++]=r;
            }
            Array.Sort(order,0,used,Comparer<int>.Create(Compare));
            for(var first=0;first<used;)
            {
                token.ThrowIfCancellationRequested();var end=first+1;
                while(end<used&&Compare(order[first],order[end])==0)end++;
                var size=end-first;histogram[size==1?0:size<8?1:size<16?2:size<64?3:4]++;
                if(size>=8)
                {
                    var row=rows.Row(begin+order[first]);eligible+=size;terms+=(long)size*row.Count;avoided+=size-1;
                    if(row.Kind==ProfileRowKind.EqualityCost)eq++;else hinge++;
                }
                first=end;
            }
        }
        return new(rows.Count,rows.Terms,rows.Variables,variables,constants,byKind,histogram,eligible,terms,avoided,eq,hinge,
            (long)order.Length*sizeof(int)+28L*sizeof(long),clock.Elapsed.TotalSeconds);
    }

    public static void Test()
    {
        var rows=new SparseProfileRows(2);
        for(var i=0;i<16;i++)rows.Append(new[]{0,1},new[]{2d,-3d},2,i,1,ProfileRowKind.UpperHingeCost,false);
        for(var i=0;i<7;i++)rows.Append(new[]{0,1},new[]{2d,-3d},2,i,2,ProfileRowKind.EqualityCost,false);
        rows.Append(new[]{0,1},new[]{2d,-3d+1e-10},2,0,1,ProfileRowKind.UpperHingeCost,false);
        rows.Append(Array.Empty<int>(),Array.Empty<double>(),0,3,2,ProfileRowKind.EqualityCost,false);rows.Freeze();
        var x=new[]{4d,5d};var before=rows.Evaluate(x,null,default);var report=Inspect(rows);
        if(report.EligibleRows!=16||report.EligibleTerms!=32||report.HingeGroups!=1||report.EqualityGroups!=0||report.ConstantRows!=1||
            rows.Count!=25||rows.Evaluate(x,null,default)!=before)throw new Exception("Row census changed data or grouped different coefficient/kind contracts.");
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        try{Inspect(rows,cancel.Token);throw new Exception("Census ignored cancellation.");}catch(OperationCanceledException){}
        Console.WriteLine("PASS: bounded exact-vector census preserves original rows and distinguishes targets, kinds and coefficients.");
    }
}
