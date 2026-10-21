using System;

namespace ImportEarthLocation.Processing;

// Four-point Gauss-Legendre on [0,1] integrates a squared cubic residual
// (degree six) exactly. This is intent cost only, never physical certification.
internal static class ProfileIntentQuadrature
{
    private static readonly double Outer=Math.Sqrt((3+2*Math.Sqrt(6d/5))/7)/2;
    private static readonly double Inner=Math.Sqrt((3-2*Math.Sqrt(6d/5))/7)/2;
    internal static readonly double[] Nodes={.5-Outer,.5-Inner,.5+Inner,.5+Outer};
    internal static readonly double[] Weights={(18-Math.Sqrt(30))/72,(18+Math.Sqrt(30))/72,(18+Math.Sqrt(30))/72,(18-Math.Sqrt(30))/72};
}
