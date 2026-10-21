using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Frozen full-width roof/terrain rows. Check leaves and terrain-cell cuts are
// analysis geometry only: none of them become native network nodes.
public static class NetworkTunnelContacts
{
    public readonly record struct Report(long Rows,int Leaves,int Uncertified,int MissingAssets,long Triangles);
    public readonly record struct CoverEvidence(int Section,double Begin,double End,long FirstRow,long RowCount,string Classification);
    private sealed record Leaf(FootprintLeaf Foot,Cubic3d Curve,int Depth);
    private sealed class Measure { public double Area; public long BoundaryRows; }

    public static Report Append(IReadOnlyList<PreparedSegment> source,ushort[] terrain,float scale,double width,
        IReadOnlyDictionary<string,NativeAssetContract> assets,int[] from,int[] c1,int[] c2,int[] to,
        SparseProfileRows rows,double epsilon,CancellationToken token,Action<string>? progress=null,Action<CoverEvidence>? evidence=null)
    {
        var n=TerrainSampling.Resolution(terrain);var cell=width/(n-1);var half=width/2;
        if(!(width>0&&scale>0&&epsilon>0))throw new ArgumentException("Invalid tunnel terrain datum.");
        var envelopes=new NativeCompositionEnvelope[source.Count];var missing=0;var leaves=new List<Leaf>();
        var nodes=new Dictionary<SourceNodeKey,int>();var surface=new HashSet<SourceNodeKey>();
        int Node(SourceNodeKey key){if(!nodes.TryGetValue(key,out var id))nodes[key]=id=nodes.Count;return id;}
        foreach(var s in source)foreach(var id in new[]{s.StartNodeId,s.EndNodeId})
        {Node(new(s.Kind,id));if(!s.Tunnel)surface.Add(new(s.Kind,id));}
        var roots=Enumerable.Range(0,nodes.Count).ToArray();
        int Root(int i){while(roots[i]!=i){roots[i]=roots[roots[i]];i=roots[i];}return i;}
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];if(!s.Tunnel)continue;
            var a=Root(Node(new(s.Kind,s.StartNodeId)));var b=Root(Node(new(s.Kind,s.EndNodeId)));roots[b]=a;
            if(!assets.TryGetValue(s.NativeAssetKey,out var asset)||!asset.TryEnvelope(NativeStructureMode.Tunnel,out envelopes[i]))
            {missing++;evidence?.Invoke(new(i,0,1,-1,0,"missing-roof-contract"));continue;}
            var c=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);
            if(!NetworkDesignChecks.Finite(c)){missing++;evidence?.Invoke(new(i,0,1,-1,0,"nonfinite-curve"));continue;}
            var initial=new List<FootprintLeaf>();NetworkFootprintMath.BuildLeaves(c,Math.Max(s.NativeWidth,envelopes[i].Width)/2,i,initial,token);
            foreach(var foot in initial)leaves.Add(new(foot,Subcurve(c,foot.Begin,foot.End),0));
        }
        var portals=surface.Where(nodes.ContainsKey).Select(k=>nodes[k]).ToArray();
        int[] start=Array.Empty<int>(),end=Array.Empty<int>();double[] upper=Array.Empty<double>(),lower=Array.Empty<double>();
        void Distances()
        {
            var high=new List<List<ProfileGraphArc>>();var low=new List<List<ProfileGraphArc>>();
            for(var i=0;i<nodes.Count;i++){high.Add(new());low.Add(new());}
            start=new int[leaves.Count];end=new int[leaves.Count];
            var last=new Dictionary<int,int>();
            for(var i=0;i<leaves.Count;i++)
            {
                var leaf=leaves[i];var s=source[leaf.Foot.Curve];
                var a=leaf.Foot.Begin==0?nodes[new(s.Kind,s.StartNodeId)]:last[leaf.Foot.Curve];
                int b;
                if(leaf.Foot.End==1)b=nodes[new(s.Kind,s.EndNodeId)];else{b=high.Count;high.Add(new());low.Add(new());}
                last[leaf.Foot.Curve]=b;start[i]=a;end[i]=b;
                var c=leaf.Curve;var chord=math.distance(c.P0.xz,c.P3.xz);
                var polygon=math.distance(c.P0.xz,c.P1.xz)+math.distance(c.P1.xz,c.P2.xz)+math.distance(c.P2.xz,c.P3.xz);
                high[a].Add(new(b,polygon));high[b].Add(new(a,polygon));low[a].Add(new(b,chord));low[b].Add(new(a,chord));
            }
            upper=new NetworkProfileGraph(high.Select(a=>a.ToArray()).ToArray()).Distances(portals,double.PositiveInfinity,token);
            lower=new NetworkProfileGraph(low.Select(a=>a.ToArray()).ToArray()).Distances(portals,double.PositiveInfinity,token);
        }
        // Refine portal-distance error, not native emission. Upper control-hull
        // path lengths and lower chords are never interchanged in cover rows.
        for(var pass=0;pass<=16;pass++)
        {
            Distances();if(pass==16)break;var refined=new List<Leaf>();var changed=false;
            for(var i=0;i<leaves.Count;i++)
            {
                token.ThrowIfCancellationRequested();var leaf=leaves[i];
                if(leaf.Depth<16&&DistanceGap(leaf,upper[start[i]],upper[end[i]],lower[start[i]],lower[end[i]])>.05)
                {
                    changed=true;var split=NetworkDesignChecks.Split(leaf.Curve,.5);var middle=(leaf.Foot.Begin+leaf.Foot.End)/2;
                    Add(split.Left,leaf.Foot.Begin,middle);Add(split.Right,middle,leaf.Foot.End);
                    void Add(Cubic3d c,double begin,double finish)
                    {
                        var parts=new List<FootprintLeaf>();var s=source[leaf.Foot.Curve];
                        NetworkFootprintMath.BuildLeaves(c,Math.Max(s.NativeWidth,envelopes[leaf.Foot.Curve].Width)/2,leaf.Foot.Curve,parts,token);
                        foreach(var part in parts)refined.Add(new(part with{Begin=begin+(finish-begin)*part.Begin,End=begin+(finish-begin)*part.End},Subcurve(c,part.Begin,part.End),leaf.Depth+1));
                    }
                }
                else refined.Add(leaf);
            }
            if(!changed)break;leaves=refined;
        }
        var measures=new Dictionary<int,Measure>();var uncertified=0;long countRows=0,triangles=0;
        var rectangle=new double2[32];var region=new double2[32];var clipped=new double2[32];var scratch=new double2[32];
        var gw=new double[6];var gc=new double[6];var delta=new double[6];var basis=new double[4];var indices=new int[4];var coefficients=new double[4];
        double Decode(int x,int z)=>terrain[z*n+x]*scale/65535d;
        void Polynomial(int x,int z,double2 a,double2 b,double2 c,double[] target)
        {var origin=new double2(x*cell-half,z*cell-half);NetworkTerrainPolynomial.Quadratic(Decode(x,z),Decode(x+1,z),Decode(x,z+1),Decode(x+1,z+1),(a-origin)/cell,(b-origin)/cell,(c-origin)/cell,target);}
        (int X,int Z) Cell(double2 p)=>((int)math.clamp(Math.Floor((p.x+half)/cell),0,n-2),(int)math.clamp(Math.Floor((p.y+half)/cell),0,n-2));
        // Optimized costs require area normalization. A fixed-vector sink only
        // checks the identical inequalities, so it needs no normalization pass.
        var measurementOnly=rows.MeasurementOnly;var unitMeasure=new Measure();
        for(var pass=measurementOnly?1:0;pass<2;pass++)for(var i=0;i<leaves.Count;i++)
        {
            var classify=pass==0||measurementOnly;
            token.ThrowIfCancellationRequested();if((i&63)==0)progress?.Invoke($"Building continuous tunnel cover; pass {(measurementOnly?1:pass+1)}/{(measurementOnly?1:2)}; footprint {i+1}/{leaves.Count}");
            var leaf=leaves[i];var foot=leaf.Foot;var edge=foot.Curve;var s=source[edge];
            var firstRow=rows.Count;
            var da=upper[start[i]];var db=upper[end[i]];var gap=DistanceGap(leaf,da,db,lower[start[i]],lower[end[i]]);
            if(!foot.Certified||gap>.05)
            {if(classify){uncertified++;evidence?.Invoke(new(edge,foot.Begin,foot.End,-1,0,"uncertified-footprint-or-portal"));}continue;}
            Measure measure;
            if(measurementOnly)measure=unitMeasure;
            else {var family=Root(nodes[new(s.Kind,s.StartNodeId)]);if(!measures.TryGetValue(family,out measure!))measures[family]=measure=new();}
            NetworkFootprintMath.Rectangle(foot,Math.Max(s.NativeWidth,envelopes[edge].Width)/2,double2.zero,rectangle);
            var breaks=new SortedSet<double>{0,1};var speed=foot.MaximumSpeed;var rampWidth=Math.Max(8,Math.Max(s.NativeWidth,envelopes[edge].Width));
            void Break(double t){if(t>0&&t<1&&SparseProfileRows.Finite(t))breaks.Add(t);}
            if(SparseProfileRows.Finite(da)&&SparseProfileRows.Finite(db))Break((db+speed-da)/(2*speed));
            foreach(var d in new[]{rampWidth,160d}){Break((d-da)/speed);Break(1-(d-db)/speed);}
            // The chord's terrain polynomial must remain in one bilinear cell.
            for(var axis=0;axis<2;axis++)
            {
                var a=foot.A[axis];var b=foot.B[axis];if(a==b)continue;
                var lo=(int)Math.Ceiling((Math.Min(a,b)+half)/cell);var hi=(int)Math.Floor((Math.Max(a,b)+half)/cell);
                for(var grid=Math.Max(0,lo);grid<=Math.Min(n-1,hi);grid++)Break((grid*cell-half-a)/(b-a));
            }
            var cuts=breaks.ToArray();
            for(var band=-1;band<cuts.Length;band++)
            {
                var lo=band<0?double.NegativeInfinity:cuts[band];var hi=band==cuts.Length-1?double.PositiveInfinity:cuts[band+1];
                var part=band<0?0:band==cuts.Length-1?2:1;
                Array.Copy(rectangle,region,4);var used=4;
                int Cut(double2[] polygon,int length,double2 normal,double threshold)
                {var used=NetworkFootprintMath.Clip(polygon,length,scratch,normal,threshold);Array.Copy(scratch,polygon,used);return used;}
                var origin=math.dot(foot.A,foot.Tangent);
                if(!double.IsNegativeInfinity(lo))used=Cut(region,used,foot.Tangent,origin+lo*foot.Length);
                if(!double.IsPositiveInfinity(hi))used=Cut(region,used,-foot.Tangent,-origin-hi*foot.Length);
                if(used==0)continue;
                double Parameter(double2 p)=>part==0?0:part==2?1:math.clamp(NetworkFootprintMath.Parameter(foot,p),0,1);
                var mid=part==0?0:part==2?1:(lo+hi)/2;
                var forward=da+speed*mid<=db+speed*(1-mid);
                double Distance(double t)=>forward?da+speed*t:db+speed*(1-t);
                var distance=Distance(mid);var interior=distance>=160;var blended=distance<rampWidth;
                var minimum=region[0];var maximum=region[0];for(var j=1;j<used;j++){minimum=math.min(minimum,region[j]);maximum=math.max(maximum,region[j]);}
                if(classify&&(minimum.x< -half||minimum.y< -half||maximum.x>half||maximum.y>half))
                {uncertified++;evidence?.Invoke(new(edge,foot.Begin,foot.End,-1,0,"outside-terrain"));}
                var ca=Cell(minimum);var cb=Cell(maximum);var centerCell=Cell(math.lerp(foot.A,foot.B,mid));
                var terrainGradient=blended?GradientBound(foot,terrain,n,scale,width):0;
                for(var z=ca.Z;z<=cb.Z;z++)for(var x=ca.X;x<=cb.X;x++)
                {
                    token.ThrowIfCancellationRequested();Array.Copy(region,clipped,used);var length=used;
                    length=Cut(clipped,length,new(1,0),x*cell-half);length=Cut(clipped,length,new(-1,0),-(x+1)*cell+half);
                    length=Cut(clipped,length,new(0,1),z*cell-half);length=Cut(clipped,length,new(0,-1),-(z+1)*cell+half);
                    if(length==0)continue;
                    for(var triangle=0;triangle<Math.Max(1,length-2);triangle++)
                    {
                        var a=clipped[0];var b=clipped[length==1?0:triangle+1];var c=clipped[length<=2?length-1:triangle+2];
                        var area=NetworkFootprintMath.TriangleArea(a,b,c);var terms=length==1?1:area==0?4:10;
                        if(area==0&&length>1)
                        {var ab=math.lengthsq(a-b);var ac=math.lengthsq(a-c);var bc=math.lengthsq(b-c);if(ac>=ab&&ac>=bc)b=c;else if(bc>ab)a=c;c=b;}
                        if(classify)triangles++;
                        if(pass==0){measure.Area+=area;if(area==0)measure.BoundaryRows+=terms;continue;}
                        var weight=measurementOnly?1:area>0?10000*area/measure.Area/10:10000d/(100*Math.Max(1,measure.BoundaryRows));
                        var ta=Parameter(a);var tb=Parameter(b);var tc=Parameter(c);
                        var pa=foot.Begin+(foot.End-foot.Begin)*ta;var pb=foot.Begin+(foot.End-foot.Begin)*tb;var pc=foot.Begin+(foot.End-foot.Begin)*tc;
                        Polynomial(x,z,a,b,c,gw);
                        if(blended)
                        {
                            Polynomial(centerCell.X,centerCell.Z,math.lerp(foot.A,foot.B,ta),math.lerp(foot.A,foot.B,tb),math.lerp(foot.A,foot.B,tc),gc);
                            for(var j=0;j<6;j++)delta[j]=gw[j]-gc[j];
                        }
                        var distances=new double3(Distance(ta),Distance(tb),Distance(tc));
                        var lambda=math.clamp(distances/rampWidth,0,1);
                        var beta=NetworkGeometrySafety.GradeLimit(s)*speed*foot.Error/foot.Length;
                        if(!interior)beta+=(22.5/160)*speed*foot.Error/foot.Length;
                        if(blended)
                        {
                            var parameterError=Math.Max(math.distance(leaf.Curve.P1.xz,(2*foot.A+foot.B)/3),math.distance(leaf.Curve.P2.xz,(foot.A+2*foot.B)/3));
                            var difference=delta.Max(v=>Math.Abs(v));
                            beta+=terrainGradient*(parameterError+foot.Error)+difference*speed*foot.Error/(foot.Length*rampWidth);
                        }
                        indices[0]=from[edge];indices[1]=c1[edge];indices[2]=c2[edge];indices[3]=to[edge];
                        for(var k=0;k<terms;k++)
                        {
                            var coefficient=terms==4?k switch{0=>0,1=>1,2=>3,_=>6}:k;
                            NetworkFootprintMath.TriangleBlossom(pa,pb,pc,coefficient,basis);
                            for(var j=0;j<4;j++)coefficients[j]=basis[j]/epsilon;
                            var ground=blended?NetworkTerrainPolynomial.Elevate(gc,coefficient)+NetworkTerrainPolynomial.AffineTimesQuadratic(lambda,delta,coefficient):NetworkTerrainPolynomial.Elevate(gw,coefficient);
                            double Cover(double p,double q,double r)=>interior?15:NetworkTerrainPolynomial.CoverBlossom(p,q,r);
                            var roofCover=CoverCoefficient(distances,coefficient,Cover);
                            rows.Append(indices,coefficients,4,(ground-roofCover-envelopes[edge].RangeMax-epsilon-beta)/epsilon,weight,ProfileRowKind.UpperHingeCost);countRows++;
                        }
                    }
                }
            }
            if(pass==1)evidence?.Invoke(new(edge,foot.Begin,foot.End,firstRow,rows.Count-firstRow,"continuous-roof-cover"));
        }
        return new(countRows,leaves.Count,uncertified,missing,triangles);
    }

    private static Cubic3d Subcurve(Cubic3d c,double begin,double end)
    {if(end<1)c=NetworkDesignChecks.Split(c,end).Left;if(begin>0)c=NetworkDesignChecks.Split(c,begin/end).Right;return c;}
    private static double DistanceGap(Leaf leaf,double da,double db,double la,double lb)
    {
        if(double.IsPositiveInfinity(la)&&double.IsPositiveInfinity(lb))return 0;
        var c=leaf.Curve;var f=leaf.Foot;var v=Math.Min(math.dot(3*(c.P1-c.P0).xz,f.Tangent),Math.Min(math.dot(3*(c.P2-c.P1).xz,f.Tangent),math.dot(3*(c.P3-c.P2).xz,f.Tangent)));
        if(!(v>0))return double.PositiveInfinity;var maximum=0d;
        foreach(var t in new[]{0d,1d,(db+f.MaximumSpeed-da)/(2*f.MaximumSpeed),(lb+v-la)/(2*v),(160-la)/v,1-(160-lb)/v})
        {
            if(!SparseProfileRows.Finite(t)||t<0||t>1)continue;
            var lower=Math.Min(la+v*t,lb+v*(1-t));if(lower>160)continue;
            maximum=Math.Max(maximum,Math.Min(da+f.MaximumSpeed*t,db+f.MaximumSpeed*(1-t))-lower);
        }
        return maximum;
    }
    private static double CoverCoefficient(double3 d,int k,Func<double,double,double,double> blossom)
        =>k switch{0=>blossom(d.x,d.x,d.x),1=>blossom(d.x,d.x,d.y),2=>blossom(d.x,d.x,d.z),3=>blossom(d.x,d.y,d.y),4=>blossom(d.x,d.y,d.z),5=>blossom(d.x,d.z,d.z),6=>blossom(d.y,d.y,d.y),7=>blossom(d.y,d.y,d.z),8=>blossom(d.y,d.z,d.z),_=>blossom(d.z,d.z,d.z)};
    private static double GradientBound(FootprintLeaf leaf,ushort[] terrain,int n,float scale,double width)
    {
        var cell=width/(n-1);var half=width/2;var extent=2*leaf.Error;
        var lo=math.min(leaf.A,leaf.B)-extent;var hi=math.max(leaf.A,leaf.B)+extent;
        var x0=(int)math.clamp(Math.Floor((lo.x+half)/cell),0,n-2);var x1=(int)math.clamp(Math.Floor((hi.x+half)/cell),0,n-2);
        var z0=(int)math.clamp(Math.Floor((lo.y+half)/cell),0,n-2);var z1=(int)math.clamp(Math.Floor((hi.y+half)/cell),0,n-2);var maximum=0d;
        for(var z=z0;z<=z1;z++)for(var x=x0;x<=x1;x++)
        {
            double H(int a,int b)=>terrain[b*n+a]*scale/65535d;
            var a=H(x,z);var b=H(x+1,z);var c=H(x,z+1);var d=H(x+1,z+1);
            foreach(var u in new[]{0d,1d})foreach(var v in new[]{0d,1d})maximum=Math.Max(maximum,math.length(new double2((b-a)*(1-v)+(d-c)*v,(c-a)*(1-u)+(d-b)*u))/cell);
        }
        return maximum;
    }
}
