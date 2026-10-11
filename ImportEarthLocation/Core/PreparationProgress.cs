using System;
using System.Diagnostics;

namespace ImportEarthLocation.Core;

// Workers only publish text here; native UI bindings are updated on the game thread.
public sealed class PreparationProgress
{
    private readonly object _gate = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private string _step = "Getting ready";

    public void Begin(string step)
    {
        lock (_gate) { _step = step; _clock.Restart(); }
    }

    // Change worker detail without resetting the elapsed time for its stage.
    public void Detail(string step)
    {
        lock(_gate) _step=step;
    }

    public string Snapshot(bool mapUnchanged = true)
    {
        lock (_gate) return $"{_step}… {_clock.Elapsed.TotalSeconds:0} seconds in this step." + (mapUnchanged ? " Your map has not changed." : " Please wait.");
    }
}
