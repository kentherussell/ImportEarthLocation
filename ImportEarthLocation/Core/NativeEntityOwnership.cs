using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

// Entity keys include their version. Only the exclusive transaction's observed
// outputs enter this registry; prefab identity or proximity never grants ownership.
public sealed class NativeEntityOwnership<T> where T : notnull
{
    private readonly HashSet<T> _retained = new();
    private readonly HashSet<T> _candidates = new();
    private readonly HashSet<T> _touched = new();
    public int Count => _retained.Count;
    public bool Contains(T entity) => _retained.Contains(entity);

    public void Capture(T preview, T original, bool hasOriginal, bool deleting, bool canceled)
    {
        _touched.Add(preview);
        if (hasOriginal) _touched.Add(original);
        if (deleting || canceled) return;
        _candidates.Add(preview);
        if (hasOriginal) _candidates.Add(original);
    }

    public void SettleApplied(Func<T, bool> isLive)
    {
        // Inspect only this batch's touched entities, not the growing world or
        // retained registry. Ordinary updates retain Original; replacements and
        // creations retain whichever proven candidates actually survive playback.
        foreach (var entity in _touched)
        {
            if (!isLive(entity)) _retained.Remove(entity);
            else if (_candidates.Contains(entity)) _retained.Add(entity);
        }
        DiscardBatch();
    }

    public void DiscardBatch() { _candidates.Clear(); _touched.Clear(); }
    public void Clear() { _retained.Clear(); DiscardBatch(); }
}
