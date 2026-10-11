using System;

namespace ImportEarthLocation.Core;

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
