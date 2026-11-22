using System.Security.Cryptography;
using System.Text;
using ImportEarthLocation.Core;
using Newtonsoft.Json;
using Unity.Mathematics;

// Exact fingerprints of a replay's stages, so a speed change can be proven to give
// byte-identical output (report 114). Floats are hashed by their bits, not as text.
static class PlanHash
{
    private sealed class VectorConverter : JsonConverter
    {
        public override bool CanRead => false;

        public override bool CanConvert(Type type)
        {
            type = Nullable.GetUnderlyingType(type) ?? type;

            return type == typeof(float2) || type == typeof(float3) || type == typeof(float4);
        }

        public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
        {
            switch (value)
            {
                case float2 point:
                    writer.WriteValue($"{Bits(point.x)},{Bits(point.y)}");
                    break;

                case float3 point:
                    writer.WriteValue($"{Bits(point.x)},{Bits(point.y)},{Bits(point.z)}");
                    break;

                case float4 point:
                    writer.WriteValue($"{Bits(point.x)},{Bits(point.y)},{Bits(point.z)},{Bits(point.w)}");
                    break;

                default:
                    writer.WriteNull();
                    break;
            }
        }

        public override object ReadJson(JsonReader reader, Type type, object? existing, JsonSerializer serializer)
            => throw new NotSupportedException();

        private static int Bits(float value) => BitConverter.SingleToInt32Bits(value);
    }

    private static readonly JsonSerializerSettings Settings = new()
    {
        Converters = { new VectorConverter() },
        FloatFormatHandling = FloatFormatHandling.String,
        FloatParseHandling = FloatParseHandling.Double
    };

    public static string Segments(IEnumerable<PreparedSegment> segments)
    {
        using var hash = SHA256.Create();
        var builder = new StringBuilder();

        foreach (var segment in segments)
        {
            builder.Clear();
            builder.Append(JsonConvert.SerializeObject(segment, Settings)).Append('\n');
            var bytes = Encoding.UTF8.GetBytes(builder.ToString());
            hash.TransformBlock(bytes, 0, bytes.Length, null, 0);
        }

        hash.TransformFinalBlock(Array.Empty<byte>(), 0, 0);

        return Convert.ToHexString(hash.Hash!);
    }

    // Everything a water plan hands on, in order: sources with their probes and repair
    // cells, area and reach stages, bed edit masks, moved waterway nodes and the report;
    // the two height grids are hashed by their bits beside it.
    public static string Water(WaterPlan plan)
    {
        var rest = plan with { ConditionedHeights = null, PlannedBedHeights = null };
        var text = JsonConvert.SerializeObject(rest, Settings);

        var parts =
            Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(text)))
            + (plan.ConditionedHeights == null ? "-" : Heights(plan.ConditionedHeights))
            + (plan.PlannedBedHeights == null ? "-" : Heights(plan.PlannedBedHeights));

        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(parts)));
    }

    public static string Heights(float[] heights)
    {
        var bytes = new byte[heights.Length * 4];
        Buffer.BlockCopy(heights, 0, bytes, 0, bytes.Length);

        return Convert.ToHexString(SHA256.HashData(bytes));
    }
}
