namespace ImportEarthLocation.Core;

public enum NativeBatchDisposition
{
    Pending, AppliedAndSettled, ClearedWithoutApply, NoPreview, Unresolved,
    // Another tool took over before Apply; the user resumed and nothing was applied.
    Interrupted
}
