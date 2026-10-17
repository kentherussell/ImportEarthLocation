using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Read-only ground-platform certificate. A requested stamp is not evidence that
// protected edits, cut/fill limits and coarse raster interpolation achieved it.
public static class NetworkTerrainSupport
{
    public static void Validate(IReadOnlyList<PreparedSegment> sections,ushort[] terrain,float scale,double width,
        ImportLog log,CancellationToken token,Action<string>? progress=null,string scope="final-design",string stage="planned",
        Action<NetworkQualityFinding>? result=null,bool publish=true)
    {
        var n=TerrainSampling.Resolution(terrain);
        if(n<2||!(width>0)||!(scale>0))
        {var unavailable=new NetworkQualityFinding(stage,"encoded-ground-support",scope,0,0,1,0,0,"Terrain registration unavailable");result?.Invoke(unavailable);if(publish)log.Quality(unavailable);return;}
        var cell=width/(n-1);var half=width/2;var epsilon=Math.Max(.05,2*scale/65535d);
        var failed=0;var unknown=0;var checkedSections=0;var maximum=0d;var affectedLength=0d;long triangles=0;
        var polygon=new double2[32];var scratch=new double2[32];var rectangle=new double2[32];var clipped=new double2[32];
        var terrainCoefficients=new double[6];var basis=new double[4];var witnesses=new List<object>();
        double Decode(int x,int z)=>terrain[z*n+x]*scale/65535d;
        int Pixel(double value)=>(int)math.clamp(Math.Floor((value+half)/cell),0,n-2);
        for(var index=0;index<sections.Count;index++)
        {
            token.ThrowIfCancellationRequested();var s=sections[index];if(NetworkStructure.IsStructure(s))continue;checkedSections++;
            if((index&63)==0)progress?.Invoke($"Checking encoded ground support {index+1:N0}/{sections.Count:N0}; {triangles:N0} terrain triangles");
            if(!(s.NativeWidth>0)){unknown++;continue;}
            var curve=new Cubic3d(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);var leaves=new List<FootprintLeaf>();
            if(!NetworkDesignChecks.Finite(curve)){unknown++;continue;}
            NetworkFootprintMath.BuildLeaves(curve,s.NativeWidth/2,index,leaves,token);
            var sampled=0d;var bound=0d;var uncertain=false;
            foreach(var leaf in leaves)
            {
                token.ThrowIfCancellationRequested();if(!leaf.Certified){uncertain=true;continue;}
                foreach(var t in new[]{leaf.Begin,(leaf.Begin+leaf.End)/2,leaf.End})
                {
                    var center=NetworkDesignChecks.Evaluate(curve,t);var direction=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                    var normal=new double2(-direction.y,direction.x);
                    foreach(var across in new[]{-s.NativeWidth/2d,0,s.NativeWidth/2d})
                    {
                        var p=center.xz+normal*across;
                        if(math.any(math.abs(p)>half)){uncertain=true;continue;}
                        var ground=TerrainSampling.Encoded(terrain,n,(p.x+half)/width,(p.y+half)/width)*scale/65535d;
                        sampled=Math.Max(sampled,Math.Abs(ground-center.y));
                    }
                }
                var vertices=NetworkFootprintMath.Rectangle(leaf,s.NativeWidth/2,default,rectangle);
                var low=leaf.Min;var high=leaf.Max;
                if(math.any(low< -half)||math.any(high>half))uncertain=true;
                var verticalSpeed=3*Math.Max(Math.Abs(curve.P1.y-curve.P0.y),Math.Max(Math.Abs(curve.P2.y-curve.P1.y),Math.Abs(curve.P3.y-curve.P2.y)));
                var beta=verticalSpeed*(leaf.End-leaf.Begin)*leaf.Error/leaf.Length;
                for(var part=0;part<3;part++)
                {
                    Array.Copy(rectangle,polygon,vertices);var count=vertices;var station=math.dot(leaf.A,leaf.Tangent);
                    int Cut(double2[] points,int used,double2 normal,double threshold)
                    {var result=NetworkFootprintMath.Clip(points,used,scratch,normal,threshold);Array.Copy(scratch,points,result);return result;}
                    if(part==0)count=Cut(polygon,count,-leaf.Tangent,-station);
                    else if(part==2)count=Cut(polygon,count,leaf.Tangent,station+leaf.Length);
                    else{count=Cut(polygon,count,leaf.Tangent,station);count=Cut(polygon,count,-leaf.Tangent,-station-leaf.Length);}
                    if(count==0)continue;
                    for(var z=Pixel(low.y);z<=Pixel(high.y);z++)for(var x=Pixel(low.x);x<=Pixel(high.x);x++)
                    {
                        token.ThrowIfCancellationRequested();Array.Copy(polygon,clipped,count);var used=count;var origin=new double2(x*cell-half,z*cell-half);
                        used=Cut(clipped,used,new(1,0),origin.x);used=Cut(clipped,used,new(-1,0),-origin.x-cell);
                        used=Cut(clipped,used,new(0,1),origin.y);used=Cut(clipped,used,new(0,-1),-origin.y-cell);
                        if(used<3)continue;
                        double Parameter(double2 p)=>leaf.Begin+(leaf.End-leaf.Begin)*(part==0?0:part==2?1:NetworkFootprintMath.Parameter(leaf,p));
                        for(var triangle=2;triangle<used;triangle++)
                        {
                            var a=clipped[0];var b=clipped[triangle-1];var c=clipped[triangle];
                            if(NetworkFootprintMath.TriangleArea(a,b,c)<=1e-12)continue;triangles++;
                            NetworkTerrainPolynomial.Quadratic(Decode(x,z),Decode(x+1,z),Decode(x,z+1),Decode(x+1,z+1),
                                (a-origin)/cell,(b-origin)/cell,(c-origin)/cell,terrainCoefficients);
                            for(var k=0;k<10;k++)
                            {
                                NetworkFootprintMath.TriangleBlossom(Parameter(a),Parameter(b),Parameter(c),k,basis);
                                var deck=basis[0]*curve.P0.y+basis[1]*curve.P1.y+basis[2]*curve.P2.y+basis[3]*curve.P3.y;
                                bound=Math.Max(bound,Math.Abs(NetworkTerrainPolynomial.Elevate(terrainCoefficients,k)-deck)+beta);
                            }
                        }
                    }
                }
            }
            maximum=Math.Max(maximum,sampled);
            if(sampled>epsilon){failed++;affectedLength+=NetworkCurveObservations.Measure(curve).SampledLengthMeters;}
            else if(uncertain||bound>epsilon)unknown++;
            if((sampled>epsilon||uncertain||bound>epsilon)&&log.DiagnosticsEnabled&&witnesses.Count<128)
                witnesses.Add(new{s.WayId,s.StartNodeId,s.EndNodeId,s.NativeUnitId,s.PairedProfileId,
                    sampledResidualMeters=sampled,continuousUpperBoundMeters=bound,toleranceMeters=epsilon,
                    state=sampled>epsilon?"PhysicalConflict":"UncertifiedGeometry",encodedRaster=true,placementBlocked=false});
        }
        var finding=new NetworkQualityFinding(stage,"encoded-ground-support",scope,checkedSections,failed,unknown,maximum,affectedLength,
            "Full-width bilinear-cell Bernstein bound; actual-curve sampled counterexample is failure; an inconclusive upper bound is unavailable");
        result?.Invoke(finding);if(publish)log.Quality(finding);
        if(publish&&log.DiagnosticsEnabled)log.DiagnosticRecords("ground-support-"+scope+".ndjson",witnesses,critical:true);
        if(publish)log.Diagnostic($"GROUND SUPPORT sections={checkedSections}; physicalConflicts={failed}; uncertifiedSections={unknown}; maximumSampledResidualMeters={maximum:R}; triangles={triangles}; fullWidth=true; protectedEditsNotOverwritten=true; placementBlocked=false");
    }
}
