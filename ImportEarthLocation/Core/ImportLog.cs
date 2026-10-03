using System;
using System.Collections.Generic;
using System.Linq;

namespace ImportEarthLocation.Core;

public sealed class ImportLog
{
    // NOTE: List-based ring buffer on purpose. Unity's mscorlib.dll (referenced by
    // the official CS2 toolchain) also defines Queue<T>, which collides with the
    // .NET Framework System.dll Queue<T> (CS0433). List<T> lives only in mscorlib
    // and compiles cleanly under net48 + Unity mscorlib.
    private readonly List<string> _lines = new();
    private readonly object _gate = new();
    private readonly SortedDictionary<string, double> _metrics = new(StringComparer.Ordinal);

    public void ResetMetrics() { lock (_gate) _metrics.Clear(); }
    public void Clear() { lock (_gate) { _lines.Clear(); _metrics.Clear(); } Changed?.Invoke(); }
    public void Count(string name, int count = 1) { lock (_gate) { _metrics.TryGetValue(name, out var value); _metrics[name] = value + count; } }
    public void Time(string name, double seconds) { lock (_gate) _metrics[name + " time (s)"] = seconds; }
    public double Metric(string name) { lock (_gate) return _metrics.TryGetValue(name, out var value) ? value : 0; }
    public string Metrics() { lock (_gate) return string.Join(" • ", _metrics.Select(item => $"{item.Key}: {item.Value:0.##}")); }

    public event Action? Changed;

    public void Info(string message) => Add("INFO", message);
    public void Warn(string message) => Add("WARN", message);
    public void Error(string message) => Add("ERROR", message);

    public string Snapshot(int maximumLines = 120)
    {
        lock (_gate)
        {
            return string.Join("\n", _lines.Skip(System.Math.Max(0, _lines.Count - maximumLines)));
        }
    }

    private void Add(string level, string message)
    {
        var line = $"{DateTime.Now:HH:mm:ss} [{level}] {message}";
        lock (_gate)
        {
            _lines.Add(line);
            while (_lines.Count > 500)
                _lines.RemoveAt(0);
        }

        if (level == "ERROR") Mod.Log.Error(line);
        else if (level == "WARN") Mod.Log.Warn(line);
        else Mod.Log.Info(line);
        Changed?.Invoke();
    }
}
