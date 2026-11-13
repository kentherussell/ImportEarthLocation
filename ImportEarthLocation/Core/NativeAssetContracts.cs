using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;

namespace ImportEarthLocation.Core;

public static class NativeAssetKeys
{
    public static bool IsHighway(PreparedSegment segment)
        => segment.Classification.StartsWith("motorway", StringComparison.OrdinalIgnoreCase)
            || segment.Classification.StartsWith("trunk", StringComparison.OrdinalIgnoreCase);

    public static PreparedSegment ApplyMetadata(
        PreparedSegment segment,
        IReadOnlyDictionary<string, float> widths,
        IReadOnlyDictionary<string, float[]> offsets)
    {
        var key = Sized(segment);
        return segment with
        {
            NativeWidth =
                widths.TryGetValue(key, out var width) && !float.IsNaN(width) && !float.IsInfinity(width) && width > 0
                    ? width
                    : 0,
            NativeLaneOffsets =
                segment.Kind == NetworkKind.Rail
                    ? Array.Empty<float>()
                    : offsets.TryGetValue(key, out var lanes) ? lanes : Array.Empty<float>()
        };
    }

    // The designer can change a section's structure mode. Its asset key and width
    // follow here; a stale key made the preflight redesign the whole map every import.
    public static IReadOnlyList<PreparedSegment> Refresh(
        IReadOnlyList<PreparedSegment> sections,
        IReadOnlyDictionary<string, NativeAssetContract> assets) => sections.Select(segment =>
    {
        var key = Sized(segment);
        var width =
            assets.TryGetValue(key, out var asset)
                && asset.TryEnvelope(NativeAssetContract.Mode(segment), out var envelope)
                ? Math.Max(segment.NativeWidth, envelope.Width)
                : segment.NativeWidth;

        return segment with { NativeAssetKey = key, NativeWidth = width };
    }).ToArray();

    public static string Family(PreparedSegment segment)
        => $"{(IsHighway(segment) ? "highway" : "road")}:{segment.OneWay}:{segment.Bridge}:{segment.ConsolidatedAvenue}";

    public static string Sized(PreparedSegment segment)
        => segment.Kind == NetworkKind.Rail
            ? $"rail:{segment.Bridge}:{Numeric.Clamp(segment.PhysicalTracks > 0 ? segment.PhysicalTracks : segment.Lanes, 1, 2)}"
            :
        Family(segment)
            + ":"
            + Numeric.Clamp(
                segment.Lanes,
                1,
                IsHighway(segment) ? (segment.OneWay ? 5 : 8) : (segment.OneWay ? NetworkSelection.OneWayRoadLanes : 6));
}

public enum NativeStructureMode : byte
{
    Ground,
    Elevated,
    Tunnel
}

// Only scalars cross the worker boundary. No Entity, native buffers or lookups.
public readonly record struct NativeCompositionEnvelope(NativeStructureMode Mode, byte SideMask,
    float Width, float SurfaceMin, float SurfaceMax, float RangeMin, float RangeMax, float NodeOffset, bool Available);

public sealed class NativeAssetContract
{
    // Effective immutable design data only. Enumeration order and live ECS
    // handles cannot invalidate an already completed design in the same epoch.
    public static string Fingerprint(IEnumerable<NativeAssetContract> assets)
    {
        using var bytes = new MemoryStream();
        using var writer = new BinaryWriter(bytes);

        foreach (var asset in assets.OrderBy(contract => contract.PaletteKey, StringComparer.Ordinal))
        {
            writer.Write(asset.PaletteKey);
            writer.Write(asset.TrainCurvatureLimit ?? float.NaN);
            var lanes = asset.LaneOffsets.OrderBy(x => x).ToArray();
            writer.Write(lanes.Length);

            foreach (var lane in lanes)
                writer.Write(lane);

            var envelopes =
                asset.Envelopes
                    .OrderBy(envelope => envelope.Mode)
                    .ThenBy(envelope => envelope.SideMask)
                    .ThenBy(envelope => envelope.Available)
                    .ThenBy(envelope => envelope.Width)
                .ThenBy(envelope => envelope.SurfaceMin)
                    .ThenBy(envelope => envelope.SurfaceMax)
                    .ThenBy(envelope => envelope.RangeMin)
                    .ThenBy(envelope => envelope.RangeMax)
                    .ThenBy(envelope => envelope.NodeOffset)
                    .ToArray();

            writer.Write(envelopes.Length);

            foreach (var envelope in envelopes)
            {
                writer.Write((byte)envelope.Mode);
                writer.Write(envelope.SideMask);
                writer.Write(envelope.Available);
                writer.Write(envelope.Width);
                writer.Write(envelope.SurfaceMin);
                writer.Write(envelope.SurfaceMax);
                writer.Write(envelope.RangeMin);
                writer.Write(envelope.RangeMax);
                writer.Write(envelope.NodeOffset);
            }
        }

        writer.Flush();
        using var hash = SHA256.Create();

        return BitConverter.ToString(hash.ComputeHash(bytes.ToArray())).Replace("-", "");
    }

    private readonly NativeCompositionEnvelope?[] _modeEnvelopes = new NativeCompositionEnvelope?[3];

    public string PaletteKey { get; }
    public IReadOnlyList<NativeCompositionEnvelope> Envelopes { get; }
    public IReadOnlyList<float> LaneOffsets { get; }
    public float? TrainCurvatureLimit { get; }

    public NativeAssetContract(
        string paletteKey,
        NativeCompositionEnvelope[] envelopes,
        IReadOnlyList<float> laneOffsets,
        float? trainCurvatureLimit = null)
    {
        TrainCurvatureLimit = trainCurvatureLimit;
        PaletteKey = paletteKey;
        Envelopes = Array.AsReadOnly((NativeCompositionEnvelope[])envelopes.Clone());
        var lanes = new float[laneOffsets.Count];

        for (var i = 0; i < lanes.Length; i++)
            lanes[i] = laneOffsets[i];

        LaneOffsets = Array.AsReadOnly(lanes);

        foreach (var mode in new[] {
            NativeStructureMode.Ground,
            NativeStructureMode.Elevated,
            NativeStructureMode.Tunnel
            })
        {
            var available = Envelopes.Where(envelope => envelope.Mode == mode && envelope.Available).ToArray();

            if (available.Length == 0)
                continue;

            _modeEnvelopes[(int)mode] =
                new(
                    mode,
                    0,
                    available.Max(envelope => envelope.Width),
                    available.Min(envelope => envelope.SurfaceMin),
                    available.Max(envelope => envelope.SurfaceMax),
                available.Min(envelope => envelope.RangeMin),
                    available.Max(envelope => envelope.RangeMax),
                    available.Max(envelope => envelope.NodeOffset),
                    true);
        }
    }

    // Earthwork sides are selected after the profile. Design against the union
    // of available side compositions in this mode, not the narrower default.
    public bool TryEnvelope(NativeStructureMode mode, out NativeCompositionEnvelope envelope)
    {
        var index = (int)mode;
        var cached = index >= 0 && index < _modeEnvelopes.Length ? _modeEnvelopes[index] : null;
        envelope = cached ?? default;

        return cached.HasValue;
    }

    public static NativeStructureMode Mode(PreparedSegment segment) => segment.Tunnel ? NativeStructureMode.Tunnel :
        segment.Bridge ? NativeStructureMode.Elevated : NativeStructureMode.Ground;

    public double? CenterlineCurvatureLimit(double width)
    {
        if (!TrainCurvatureLimit.HasValue)
            return null;

        var k = (double)TrainCurvatureLimit.Value;

        if (double.IsNaN(k) || double.IsInfinity(k) || k <= 0)
            return null;

        var distance = LaneOffsets.Count > 0 ? LaneOffsets.Max(v => Math.Abs((double)v)) : width / 2;

        return k / (1 + k * distance);
    }
}
