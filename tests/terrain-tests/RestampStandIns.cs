// Only compile the readiness/rollback harness against the expanded bridge API.
// Its synthetic engine MVID must reject restamping before any of these hooks.
// Actual native signatures are verified by the separate game-reference build.
namespace ImportEarthLocation.Processing
{
    public static class NetworkRoadbed
    {
        public sealed record Patch(Unity.Mathematics.float2 Minimum,Unity.Mathematics.float2 Maximum)
        {
            public double Encoded(double x,double z)=>throw new NotSupportedException("Roadbed publication is not simulated");
        }
    }
}
namespace Colossal.Mathematics
{
    public readonly record struct Bounds2(Unity.Mathematics.float2 min,Unity.Mathematics.float2 max);
}
namespace Unity.Jobs
{
    public struct JobHandle
    {
        public void Complete()=>throw new NotSupportedException("Native jobs are not simulated");
    }
}
