using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace ImportEarthLocation.Data;

public sealed class CacheStore
{
    public string Root { get; }

    public CacheStore()
    {
        Root = Path.Combine(Application.persistentDataPath, "ModsData", Mod.Id, "cache");
        Directory.CreateDirectory(Root);
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
        var path = PathFor(group, key, extension);
        if (!File.Exists(path)) return null;
        var bytes = await Task.Run(() => File.ReadAllBytes(path), cancellationToken).ConfigureAwait(false);
        if (validator(bytes)) return bytes;
        File.Delete(path);
        return null;
    }

    public async Task WriteAsync(string group, string key, string extension, byte[] bytes, CancellationToken cancellationToken)
    {
        var path = PathFor(group, key, extension);
        var temporary = path + ".download";
        await Task.Run(() => File.WriteAllBytes(temporary, bytes), cancellationToken).ConfigureAwait(false);
        if (File.Exists(path)) File.Delete(path);
        File.Move(temporary, path);
    }

    private static string Hash(string value)
    {
        using var sha = SHA256.Create();
        var bytes = sha.ComputeHash(Encoding.UTF8.GetBytes(value));
        return BitConverter.ToString(bytes).Replace("-", "").ToLowerInvariant();
    }
}
