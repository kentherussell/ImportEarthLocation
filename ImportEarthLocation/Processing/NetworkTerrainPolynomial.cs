using System;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Exact bilinear terrain restricted to a triangle, elevated to degree three.</summary>
public static class NetworkTerrainPolynomial
{
    // Quadratic order: (2,0,0),(1,1,0),(1,0,1),(0,2,0),(0,1,1),(0,0,2).
    public static void Quadratic(double g00,double g10,double g01,double g11,double2 a,double2 b,double2 c,double[] gamma)
    {
        var gx=g10-g00;var gz=g01-g00;var gxz=g11-g10-g01+g00;
        double Polar(double2 p,double2 q)=>g00+gx*(p.x+q.x)/2+gz*(p.y+q.y)/2+gxz*(p.x*q.y+q.x*p.y)/2;
        gamma[0]=Polar(a,a);gamma[1]=Polar(a,b);gamma[2]=Polar(a,c);
        gamma[3]=Polar(b,b);gamma[4]=Polar(b,c);gamma[5]=Polar(c,c);
    }
    private static double Coefficient(double[] gamma,int i,int j,int k)
    {
        if(i==2)return gamma[0];if(i==1&&j==1)return gamma[1];if(i==1&&k==1)return gamma[2];
        if(j==2)return gamma[3];if(j==1&&k==1)return gamma[4];if(k==2)return gamma[5];
        throw new ArgumentException("Invalid degree-two multi-index.");
    }
    private static void MultiIndex(int coefficient,out int i,out int j,out int k)
    {
        switch(coefficient)
        {
            case 0:i=3;j=0;k=0;return;case 1:i=2;j=1;k=0;return;case 2:i=2;j=0;k=1;return;
            case 3:i=1;j=2;k=0;return;case 4:i=1;j=1;k=1;return;case 5:i=1;j=0;k=2;return;
            case 6:i=0;j=3;k=0;return;case 7:i=0;j=2;k=1;return;case 8:i=0;j=1;k=2;return;
            case 9:i=0;j=0;k=3;return;default:throw new ArgumentOutOfRangeException(nameof(coefficient));
        }
    }
    public static double Elevate(double[] gamma,int coefficient)
    {
        MultiIndex(coefficient,out var i,out var j,out var k);var result=0d;
        if(i>0)result+=i*Coefficient(gamma,i-1,j,k);
        if(j>0)result+=j*Coefficient(gamma,i,j-1,k);
        if(k>0)result+=k*Coefficient(gamma,i,j,k-1);
        return result/3;
    }
    public static double AffineTimesQuadratic(double3 affine,double[] gamma,int coefficient)
    {
        MultiIndex(coefficient,out var i,out var j,out var k);var result=0d;
        if(i>0)result+=i*affine.x*Coefficient(gamma,i-1,j,k);
        if(j>0)result+=j*affine.y*Coefficient(gamma,i,j-1,k);
        if(k>0)result+=k*affine.z*Coefficient(gamma,i,j,k-1);
        return result/3;
    }
    public static double CoverBlossom(double p,double q,double r)
        =>15*((p*q+p*r+q*r)/(160*160d)-2*p*q*r/(160*160*160d));
    public static double EvaluateCubic(double[] coefficients,double3 barycentric)
    {
        var a=barycentric.x;var b=barycentric.y;var c=barycentric.z;
        return coefficients[0]*a*a*a+3*coefficients[1]*a*a*b+3*coefficients[2]*a*a*c+
            3*coefficients[3]*a*b*b+6*coefficients[4]*a*b*c+3*coefficients[5]*a*c*c+
            coefficients[6]*b*b*b+3*coefficients[7]*b*b*c+3*coefficients[8]*b*c*c+coefficients[9]*c*c*c;
    }
}
