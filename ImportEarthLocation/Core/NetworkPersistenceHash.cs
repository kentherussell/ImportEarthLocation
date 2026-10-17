using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using System.Text;

namespace ImportEarthLocation.Core;

public static class NetworkPersistenceHash
{
    // Length-prefix UTF8 records so separators in asset names cannot collide.
    // Callers supply semantic geometry/incidence, never transient entity IDs.
    public static string Digest(IEnumerable<string> records)
    {
        using var hash=SHA256.Create();
        foreach(var record in records.OrderBy(s=>s,StringComparer.Ordinal))
        {
            var data=Encoding.UTF8.GetBytes(record);var length=BitConverter.GetBytes(data.Length);
            hash.TransformBlock(length,0,length.Length,length,0);hash.TransformBlock(data,0,data.Length,data,0);
        }
        hash.TransformFinalBlock(Array.Empty<byte>(),0,0);
        return BitConverter.ToString(hash.Hash!).Replace("-","");
    }
}
