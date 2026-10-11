using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

/// <summary>Content-addressed merged data. XML remains the authority if a derived cache is unavailable.</summary>
public static class OsmParsedCache
{
    private const int Version = 3;

    public static OsmDataset Merge(IReadOnlyList<byte[]> documents, string cacheRoot, ImportLog log, CancellationToken token, bool wholeMap = false)
    {
        token.ThrowIfCancellationRequested();
        if (documents.Count != (wholeMap ? 1 : 16)) throw new InvalidDataException("Complete whole-map or 16-tile OSM coverage is required.");
        var clock = Stopwatch.StartNew();
        string path;
        using (var sha = SHA256.Create())
        {
            var key = new StringBuilder("parsed-osm-v" + Version);
            foreach (var document in documents)
            { token.ThrowIfCancellationRequested(); key.Append(':').Append(BitConverter.ToString(sha.ComputeHash(document))); }
            path = Path.Combine(cacheRoot, "parsed-osm", BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(key.ToString()))).Replace("-", "") + ".bin");
        }
        try
        {
            if (File.Exists(path))
            {
                var envelope = File.ReadAllBytes(path);
                token.ThrowIfCancellationRequested();
                if (envelope.Length < 48) throw new InvalidDataException("Truncated parsed cache.");
                using var sha = SHA256.Create();
                if (!sha.ComputeHash(envelope, 32, envelope.Length - 32).SequenceEqual(envelope.Take(32)))
                    throw new InvalidDataException("Parsed cache checksum mismatch.");
                using var stream = new MemoryStream(envelope, 32, envelope.Length - 32, writable: false);
                using var reader = new BinaryReader(stream);
                var result = Read(reader, token);
                if (stream.Position != stream.Length) throw new InvalidDataException("Trailing parsed cache data.");
                log.Count("Parsed OSM cache hits");
                log.Count("Malformed OSM features skipped", result.MalformedFeaturesSkipped);
                log.Info($"Parsed OSM cache hit: {result.Nodes.Count:N0} nodes, {result.Ways.Count:N0} ways, {result.Relations.Count:N0} relations in {clock.Elapsed.TotalSeconds:0.###}s.");
                log.Diagnostic($"PARSED OSM CACHE hit=true; nodes={result.Nodes.Count}; ways={result.Ways.Count}; relations={result.Relations.Count}; bytes={envelope.Length}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}");
                return result;
            }
        }
        catch (Exception exception) when (CacheFailure(exception))
        { log.Warn("Parsed OSM cache unavailable; reparsing original XML: " + exception.Message); }

        var parsed = OsmParser.Merge(documents, log, token, wholeMap);
        var temporary = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            using var payload = new MemoryStream();
            using (var writer = new BinaryWriter(payload, Encoding.UTF8, leaveOpen: true)) Write(writer, parsed, token);
            var bytes = payload.ToArray();
            using var sha = SHA256.Create();
            using (var output = File.Create(temporary))
            { var digest = sha.ComputeHash(bytes); output.Write(digest, 0, digest.Length); output.Write(bytes, 0, bytes.Length); }
            token.ThrowIfCancellationRequested();
            if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path);
            log.Count("Parsed OSM cache writes");
            log.Diagnostic($"PARSED OSM CACHE hit=false; stored=true; nodes={parsed.Nodes.Count}; ways={parsed.Ways.Count}; relations={parsed.Relations.Count}; bytes={bytes.Length+32}; elapsedSeconds={clock.Elapsed.TotalSeconds:0.###}");
        }
        catch (Exception exception) when (CacheFailure(exception))
        { log.Warn("Parsed OSM cache write skipped; parsed import data retained: " + exception.Message); }
        finally
        {
            try { if (File.Exists(temporary)) File.Delete(temporary); }
            catch (Exception exception) when (CacheFailure(exception)) { log.Warn("Parsed cache temporary cleanup skipped: " + exception.Message); }
        }
        token.ThrowIfCancellationRequested();
        log.Time("Parsed OSM cache miss/parse/write", clock.Elapsed.TotalSeconds);
        return parsed;
    }

    private static bool CacheFailure(Exception e) => e is InvalidDataException || e is IOException || e is UnauthorizedAccessException || e is ArgumentException || e is FormatException || e is OverflowException;

    private static void Write(BinaryWriter writer, OsmDataset data, CancellationToken token)
    {
        writer.Write(Version); writer.Write(data.MalformedFeaturesSkipped); writer.Write(data.Nodes.Count);
        foreach (var node in data.Nodes.Values)
        {
            token.ThrowIfCancellationRequested(); writer.Write(node.Id); writer.Write(node.Latitude); writer.Write(node.Longitude);
            if (data.NodeTags.TryGetValue(node.Id, out var tags)) WriteTags(writer, tags); else writer.Write(0);
        }
        writer.Write(data.Ways.Count);
        foreach (var way in data.Ways.Values)
        {
            token.ThrowIfCancellationRequested(); writer.Write(way.Id); writer.Write(way.NodeIds.Length);
            foreach (var id in way.NodeIds) writer.Write(id);
            WriteTags(writer, way.Tags);
        }
        writer.Write(data.Relations.Count);
        foreach (var relation in data.Relations)
        {
            token.ThrowIfCancellationRequested(); writer.Write(relation.Id);
            var members=relation.OrderedMembers.Count>0?relation.OrderedMembers:relation.Members.Select(m=>new OsmMember("way",m.wayId,m.inner?"inner":"outer")).ToArray();
            writer.Write(members.Count);
            foreach (var member in members) { writer.Write(member.Type); writer.Write(member.Id); writer.Write(member.Role); }
            WriteTags(writer, relation.Tags);
        }
    }

    private static void WriteTags(BinaryWriter writer, IReadOnlyDictionary<string, string> tags)
    { writer.Write(tags.Count); foreach (var tag in tags) { writer.Write(tag.Key); writer.Write(tag.Value); } }

    private static int Count(BinaryReader reader, int minimumBytes)
    {
        var count = reader.ReadInt32();
        if (count < 0 || count > (reader.BaseStream.Length - reader.BaseStream.Position) / minimumBytes)
            throw new InvalidDataException("Invalid parsed OSM count.");
        return count;
    }

    private static Dictionary<string, string> Tags(BinaryReader reader, int? knownCount = null)
    {
        var tags = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        var count = knownCount ?? Count(reader, 2);
        for (var i = 0; i < count; i++) tags[reader.ReadString()] = reader.ReadString();
        return tags;
    }

    private static OsmDataset Read(BinaryReader reader, CancellationToken token)
    {
        if (reader.ReadInt32() != Version) throw new InvalidDataException("Unsupported parsed OSM cache version.");
        var skipped = reader.ReadInt32();
        if (skipped < 0) throw new InvalidDataException("Invalid skipped-feature count.");
        var nodes = new Dictionary<long, OsmNode>(); var ways = new Dictionary<long, OsmWay>(); var relations = new List<OsmRelation>();
        var nodeTags = new Dictionary<long, IReadOnlyDictionary<string, string>>();
        var count = Count(reader, 28);
        for (var i = 0; i < count; i++)
        {
            token.ThrowIfCancellationRequested(); var id = reader.ReadInt64(); nodes.Add(id, new OsmNode(id, reader.ReadDouble(), reader.ReadDouble()));
            var tagCount = Count(reader, 2); if (tagCount > 0) nodeTags.Add(id, Tags(reader, tagCount));
        }
        count = Count(reader, 16);
        for (var i = 0; i < count; i++)
        {
            token.ThrowIfCancellationRequested(); var id = reader.ReadInt64(); var refs = new long[Count(reader, 8)];
            for (var j = 0; j < refs.Length; j++) refs[j] = reader.ReadInt64();
            ways.Add(id, new OsmWay(id, refs, Tags(reader)));
        }
        count = Count(reader, 16);
        for (var i = 0; i < count; i++)
        {
            token.ThrowIfCancellationRequested(); var id = reader.ReadInt64(); var ordered = new OsmMember[Count(reader, 10)];
            for (var j = 0; j < ordered.Length; j++) ordered[j] = new OsmMember(reader.ReadString(),reader.ReadInt64(),reader.ReadString());
            relations.Add(new OsmRelation(id, ordered.Where(m=>m.Type=="way").Select(m=>(m.Id,m.Role=="inner")).ToArray(), Tags(reader)) { OrderedMembers=ordered });
        }
        return new OsmDataset { Nodes = nodes, NodeTags = nodeTags, Ways = ways, Relations = relations, MalformedFeaturesSkipped = skipped };
    }
}
