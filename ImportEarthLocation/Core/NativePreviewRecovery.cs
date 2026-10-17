using System;

namespace ImportEarthLocation.Core;

// A quality observer may not downgrade missing native structure or ambiguous
// entity replacement contracts to RequiresReview and proceed with Apply.
public sealed class NativeSafetyException : InvalidOperationException
{
    public NativeSafetyException(string message):base(message){}
}

// Quality-driven, once-only repair request. It is neither a fatal native
// safety exception nor permission to replay an already issued Apply.
public sealed class NativeRefitRequest : InvalidOperationException
{
    public NativeRefitRequest():base("One owned unapplied unit refit requested against verified native boundary poses."){}
}

public static class NativePreviewSafety
{
    public static void RequireEdge(bool validNodes,bool hasPrefab,bool hasCurve,bool finiteGeometry)
    {
        if(!validNodes)throw new NativeSafetyException("Native edge has invalid or identical node references.");
        if(!hasPrefab||!hasCurve)throw new NativeSafetyException("Native edge is missing its required prefab/curve contract.");
        if(!finiteGeometry)throw new NativeSafetyException("Native node or curve geometry is nonfinite.");
    }
}

// Only a rejected, uncommitted graph can be regenerated. Never replay Apply
// after a timeout/readback failure: that can duplicate already-live networks.
public sealed class NativePreviewException : InvalidOperationException
{
    public long WayId { get; }
    public string[] Sources { get; }
    public NativePreviewException(string message,long wayId,params string[] sources):base(message)
    {WayId=wayId;Sources=sources;}
}

public static class NativePreviewRecovery
{
    public const int MaximumAttempts=3;
    public static int NextBatchSize(int count,int attempts,bool applied,bool recoverable)
    {
        if(count<=1||attempts<0||attempts>=MaximumAttempts||applied||!recoverable)return 0;
        // Change the input; do not repeat an identical single-section preview.
        return attempts==0?Math.Max(1,count/4):1;
    }
}
