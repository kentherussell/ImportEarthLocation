using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using ImportEarthLocation.Core;
using UnityEngine;

namespace ImportEarthLocation.Data;

public sealed class CacheStore
{
    public string Root { get; }
    private readonly ImportLog? _log;
    private static readonly object PublishGate = new();

    public CacheStore(ImportLog? log = null)
        : this(Path.Combine(Application.persistentDataPath, "ModsData", Mod.Id, "cache"), log) { }

    internal CacheStore(string root, ImportLog? log = null)
    {
        Root = root;
        _log = log;
        // A cache is an optimization, not a prerequisite for an import. Create
        // directories inside the guarded read/write operations, not here.
    }

    public string PathFor(string group, string key, string extension)
    {
        var directory = Path.Combine(Root, group);
        Directory.CreateDirectory(directory);
        return Path.Combine(directory, $"{Hash(key)}.{extension.TrimStart('.')}");
    }

    public async Task<byte[]?> ReadValidAsync(
        string group,
        string key,
        string extension,
        Func<byte[], bool> validator,
        CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        try
        {
            var path = PathFor(group, key, extension);
            if (!File.Exists(path)) return null;
            var bytes = await Task.Run(() => File.ReadAllBytes(path), cancellationToken).ConfigureAwait(false);
            cancellationToken.ThrowIfCancellationRequested();
            if (validator(bytes)) return bytes;
            // Retain invalid bytes for diagnosis; never serve them to a caller.
            await QuarantineAsync(group, key, extension, cancellationToken).ConfigureAwait(false);
            _log?.Diagnostic($"CACHE invalid group={group}; action=quarantine; freshDownloadAllowed=true");
            return null;
        }
        catch (Exception error) when (DiskFailure(error))
        {
            Report(group, "read", error);
            return null;
        }
    }

    public async Task WriteAsync(string group, string key, string extension, byte[] bytes, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        string? temporary = null;
        try
        {
            var path = PathFor(group, key, extension);
            temporary = path + "." + Guid.NewGuid().ToString("N") + ".download";
            await Task.Run(() => File.WriteAllBytes(temporary, bytes), cancellationToken).ConfigureAwait(false);
            cancellationToken.ThrowIfCancellationRequested();
            // Independent writers have unique staging files. Publish atomically
            // without deleting the previous good cache before a successful move.
            lock (PublishGate)
            {
                if (File.Exists(path)) File.Replace(temporary, path, null);
                else File.Move(temporary, path);
            }
        }
        catch (Exception error) when (DiskFailure(error)) { Report(group, "write", error); }
        finally
        {
            if (temporary != null)
                try { if (File.Exists(temporary)) File.Delete(temporary); }
                catch (Exception error) when (DiskFailure(error)) { Report(group, "temporary-cleanup", error); }
        }
        cancellationToken.ThrowIfCancellationRequested();
    }

    public Task QuarantineAsync(string group,string key,string extension,CancellationToken token)
    {
        var path=PathFor(group,key,extension);
        return Task.Run(()=>{token.ThrowIfCancellationRequested();if(File.Exists(path))File.Move(path,path+".invalid-"+Guid.NewGuid().ToString("N"));},token);
    }

    private static bool DiskFailure(Exception error) => error is IOException || error is UnauthorizedAccessException || error is System.Security.SecurityException;
    private void Report(string group, string operation, Exception error)
    {
        _log?.Warn($"{group} cache {operation} unavailable ({error.GetType().Name}); original/downloaded data retained, import can continue without caching.");
        _log?.Diagnostic($"CACHE unavailable group={group}; operation={operation}; exception={error.GetType().Name}; cacheOptional=true");
    }

    private static string Hash(string value)
    {
        using var sha = SHA256.Create();
        var bytes = sha.ComputeHash(Encoding.UTF8.GetBytes(value));
        return BitConverter.ToString(bytes).Replace("-", "").ToLowerInvariant();
    }
}
