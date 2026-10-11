using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Newtonsoft.Json;

namespace ImportEarthLocation.Core;

// Local, append-only run history. No uploads, background services, or native game access.
public sealed class RunArchive
{
    public string DirectoryPath { get; }
    private readonly DateTime _started = DateTime.UtcNow;
    private string _state = "preparing";
    private long _checkpointSequence;

    public sealed record ArtifactResult(string File, int Records, long Bytes, bool Truncated);

    public RunArchive(string root, object settings, string assemblyPath, object? environment = null)
    {
        DirectoryPath = Path.Combine(root, _started.ToString("yyyyMMddTHHmmssfffZ") + "-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(DirectoryPath);
        string? fingerprint = null;
        var fingerprintStatus = "assembly-location-unavailable";
        if (File.Exists(assemblyPath))
        {
            // Fingerprinting must not abort the complete diagnostic archive if
            // a loader exposes an inaccessible or transient assembly location.
            try
            {
                using var sha = SHA256.Create(); using var stream = File.OpenRead(assemblyPath);
                fingerprint = BitConverter.ToString(sha.ComputeHash(stream)).Replace("-", "").ToLowerInvariant();
                fingerprintStatus = "loaded-assembly-file-hashed";
            }
            catch(IOException){fingerprintStatus="assembly-file-unreadable";}
            catch(UnauthorizedAccessException){fingerprintStatus="assembly-file-access-denied";}
        }
        File.WriteAllText(Path.Combine(DirectoryPath, "run.json"), JsonConvert.SerializeObject(new
        {
            schemaVersion = 2, pipelineVersion = "owned-route-overlay-v2", startedUtc = _started,
            assemblySha256 = fingerprint, assemblyFingerprintStatus = fingerprintStatus,
            assemblyFileName = string.IsNullOrEmpty(assemblyPath)?null:Path.GetFileName(assemblyPath), settings, environment,
            evidence = "Recorded operations and counts are not proof of final native rendering or persistence. Read native readback messages separately."
        }, Formatting.Indented), Encoding.UTF8);
    }

    public void Write(IReadOnlyList<string> lines, IReadOnlyDictionary<string, double> metrics, string? state)
    {
        if (state != null) _state = state;
        if (lines.Count > 0) File.AppendAllText(Path.Combine(DirectoryPath, "events.log"), string.Join(Environment.NewLine, lines) + Environment.NewLine, Encoding.UTF8);
        var path = Path.Combine(DirectoryPath, "metrics.json");
        var temporary = path + ".tmp";
        File.WriteAllText(temporary, JsonConvert.SerializeObject(new
        {
            schemaVersion = 1, startedUtc = _started, updatedUtc = DateTime.UtcNow, state = _state,
            elapsedSeconds = (DateTime.UtcNow - _started).TotalSeconds, metrics
        }, Formatting.Indented), Encoding.UTF8);
        if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path);
    }

    // Written BEFORE entering a native operation, rather than relying on the
    // periodic event buffer surviving an abrupt process exit. Not a crash diagnosis.
    public void Checkpoint(string phase, object details)
    {
        var journal = ArtifactPath("checkpoints.ndjson");
        var journalBytes = File.Exists(journal) ? new FileInfo(journal).Length : 0;
        var sequence = ++_checkpointSequence;
        object Record(bool truncated) => new
        {
            schemaVersion = 1, sequence, recordedUtc = DateTime.UtcNow,
            phase, state = _state, details,
            checkpointJournalTruncated = truncated,
            evidence = "Last recorded operation, not proof it crashed or completed. Native faults can occur asynchronously."
        };
        var bytes = new UTF8Encoding(false).GetBytes(JsonConvert.SerializeObject(Record(false)) + "\n");
        var canAppend = journalBytes + bytes.Length <= 8L * 1024 * 1024;
        WriteJson("last-operation.json", Record(!canAppend), durable: true);
        if (canAppend)
        {
            using var stream = new FileStream(journal, FileMode.Append, FileAccess.Write, FileShare.Read); stream.Write(bytes, 0, bytes.Length);
        }
    }

    public void WriteJson(string name, object value, bool durable = false)
    {
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            using (var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, leaveOpen: true))
            {
                new JsonSerializer { FloatFormatHandling = FloatFormatHandling.String }.Serialize(writer, value);
                writer.Flush();
            }
            stream.Flush(durable);
        }
        Publish(temporary, path);
    }

    public ArtifactResult WriteRecords(string name, IEnumerable<object> records, long maximumBytes = 64L * 1024 * 1024, int maximumRecords = 250000)
    {
        if (maximumBytes < 1 || maximumRecords < 1) throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        var count = 0; long bytes = 0; var truncated = false; var encoding = new UTF8Encoding(false);
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            foreach (var record in records)
            {
                var line = JsonConvert.SerializeObject(record, new JsonSerializerSettings { FloatFormatHandling = FloatFormatHandling.String }) + "\n";
                var size = encoding.GetByteCount(line);
                if (count >= maximumRecords || bytes + size > maximumBytes) { truncated = true; break; }
                var buffer = encoding.GetBytes(line); stream.Write(buffer, 0, buffer.Length);
                count++; bytes += buffer.Length;
            }
        }
        Publish(temporary, path);
        var result = new ArtifactResult(name, count, bytes, truncated);
        WriteJson(name + ".meta.json", result);
        return result;
    }

    public void WriteBytes(string name, byte[] values, int maximumBytes = 32 * 1024 * 1024)
    {
        if (values.Length > maximumBytes) throw new InvalidOperationException("Diagnostic raster exceeds its byte budget.");
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read)) stream.Write(values, 0, values.Length);
        Publish(temporary, path);
    }

    public void WriteHeights(string name, ushort[] heights)
    {
        if (heights.Length > 16_777_216) throw new InvalidOperationException("Diagnostic height raster exceeds its byte budget.");
        var path = ArtifactPath(name); var temporary = path + ".tmp";
        using (var stream = new FileStream(temporary, FileMode.Create, FileAccess.Write, FileShare.Read))
        {
            if (BitConverter.IsLittleEndian)
            {
                var buffer = new byte[65536]; var total = heights.Length * 2;
                for (var offset = 0; offset < total; offset += buffer.Length)
                { var length = Math.Min(buffer.Length, total - offset); Buffer.BlockCopy(heights, offset, buffer, 0, length); stream.Write(buffer, 0, length); }
            }
            else { using var writer = new BinaryWriter(stream); foreach (var value in heights) writer.Write(value); }
        }
        Publish(temporary, path);
    }

    private string ArtifactPath(string name)
    {
        if (string.IsNullOrEmpty(name) || name.Contains("..") || name != Path.GetFileName(name)) throw new ArgumentException("Diagnostic filename must stay inside its run directory.");
        foreach (var c in name) if (!char.IsLetterOrDigit(c) && c != '-' && c != '_' && c != '.') throw new ArgumentException("Invalid diagnostic filename.");
        return Path.Combine(DirectoryPath, name);
    }

    private static void Publish(string temporary, string path)
    { if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path); }
}
