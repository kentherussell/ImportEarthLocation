using System.IO.Compression;
using System.Text;

// Minimal read-only loose-object reader (no git binary on this machine).
public static class GitRead
{
    public const string Repo = @"C:\Users\kenru\source\repos\ImportEarthLocation\.git";

    static byte[] Obj(string hash)
    {
        var path = Path.Combine(Repo, "objects", hash.Substring(0, 2), hash.Substring(2));
        using var stream = File.OpenRead(path);
        using var otherStream = new ZLibStream(stream, CompressionMode.Decompress);
        using var ms = new MemoryStream();
        otherStream.CopyTo(ms);

        return ms.ToArray();
    }

    static (string type, byte[] body) Parse(byte[] raw)
    {
        var i = Array.IndexOf(raw, (byte)0);
        var head = Encoding.ASCII.GetString(raw, 0, i);

        return (head.Split(' ')[0], raw[(i + 1)..]);
    }

    public static string FindCommit(string prefix)
    {
        var direction = Path.Combine(Repo, "objects", prefix.Substring(0, 2));
        var text =
            Directory
                .GetFiles(direction)
                .Select(Path.GetFileName)
                .First(otherText => otherText!.StartsWith(prefix.Substring(2)));

        return prefix.Substring(0, 2) + text;
    }

    public static Dictionary<string, string> Tree(string commitPrefix)
    {
        var (otherText, b) = Parse(Obj(FindCommit(commitPrefix)));
        var text = Encoding.UTF8.GetString(b);
        var tree = text.Split('\n')[0].Split(' ')[1];
        var map = new Dictionary<string, string>();
        Walk(tree, "", map);

        return map;
    }

    static void Walk(string otherHash, string prefix, Dictionary<string, string> map)
    {
        var (text, b) = Parse(Obj(otherHash));
        var i = 0;

        while (i < b.Length)
        {
            var sp = Array.IndexOf(b, (byte)' ', i);
            var mode = Encoding.ASCII.GetString(b, i, sp - i);
            var nul = Array.IndexOf(b, (byte)0, sp);
            var name = Encoding.UTF8.GetString(b, sp + 1, nul - sp - 1);
            var hash = Convert.ToHexString(b, nul + 1, 20).ToLowerInvariant();
            i = nul + 21;

            if (mode == "40000")
                Walk(hash, prefix + name + "/", map);
            else
                map[prefix + name] = hash;
        }
    }

    public static byte[] Blob(string hash) => Parse(Obj(hash)).body;

    public static void Export(string commitPrefix, string outDir, string filter)
    {
        var lookup = Tree(commitPrefix);
        var n = 0;

        foreach (var kv in lookup)
        {
            if (!kv.Key.StartsWith(filter))
                continue;

            var text = Path.Combine(outDir, kv.Key.Replace('/', '\\'));
            Directory.CreateDirectory(Path.GetDirectoryName(text)!);
            File.WriteAllBytes(text, Blob(kv.Value));
            n++;
        }

        Console.WriteLine($"exported {n} files of {lookup.Count} from {commitPrefix} to {outDir}");
    }

    public static void Diff(string firstText, string secondText, string filter)
    {
        var ta = Tree(firstText);
        var tb = Tree(secondText);

        foreach (var text in ta.Keys
            .Union(tb.Keys)
            .Where(otherText => otherText.StartsWith(filter))
            .OrderBy(otherText => otherText))
        {
            ta.TryGetValue(text, out var otherText);
            tb.TryGetValue(text, out var thirdText);

            if (otherText == thirdText)
                continue;

            int LineCount(string? fourthText)
                => fourthText == null ? 0 : Encoding.UTF8.GetString(Blob(fourthText)).Split('\n').Length;

            Console.WriteLine(
                $"{(otherText == null ? "A" : thirdText == null ? "D" : "M")} {LineCount(otherText),6}->{LineCount(thirdText),6} {text}");
        }
    }
}
