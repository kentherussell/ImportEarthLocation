using System.Threading;

namespace ImportEarthLocation.Core;

/// <summary>Process-wide epoch fences static queues across editor maps and mod/world reloads.</summary>
public static class ImportSession
{
    private static long s_Map = 1, s_Generation;
    private static int s_Ready = 1;
    public static long Map => Interlocked.Read(ref s_Map);
    public static long Generation => Interlocked.Read(ref s_Generation);
    public static bool Ready => Volatile.Read(ref s_Ready) == 1;
    public static long Begin() => Interlocked.Increment(ref s_Generation);
    public static void InvalidateOperation() => Interlocked.Increment(ref s_Generation);
    public static void ChangeMap(bool ready)
    { Volatile.Write(ref s_Ready, 0); Interlocked.Increment(ref s_Map); InvalidateOperation(); Volatile.Write(ref s_Ready, ready ? 1 : 0); }
    public static bool Matches(long map, long generation) => Ready && map == Map && generation == Generation;
}
