using System;
using System.IO;
using ImportEarthLocation.Core;
using Newtonsoft.Json.Linq;

namespace ImportEarthLocation.Data;

// The endpoint contains a personal credential. Persist locally, never include it in diagnostics.
public static class ServicePreferences
{
    private static string FilePath => Path.Combine(UnityEngine.Application.persistentDataPath, "ModsSettings", Mod.Id, "service-settings.json");
    public static string Normalize(string? input)
    {
        if (string.IsNullOrWhiteSpace(input)) return "";
        if (!Uri.TryCreate(input!.Trim(), UriKind.Absolute, out var uri) || uri.Scheme != "https" ||
            !string.Equals(uri.Host, "overpass.nextgis.com", StringComparison.OrdinalIgnoreCase) ||
            !uri.IsDefaultPort || uri.UserInfo.Length != 0 || uri.Query.Length != 0 || uri.Fragment.Length != 0)
            throw new ArgumentException("Paste your HTTPS NextGIS Overpass service link, not your profile URL or API key alone.");
        var path = uri.AbsolutePath.TrimEnd('/');
        if (path.EndsWith("/api/interpreter", StringComparison.Ordinal)) path = path.Substring(0, path.Length - 16);
        else if (path.EndsWith("/api", StringComparison.Ordinal)) path = path.Substring(0, path.Length - 4);
        if (path.Length < 2 || path.Substring(1).Contains("/") || path.Contains("%"))
            throw new ArgumentException("The service link must contain your personal Overpass service identifier.");
        return "https://overpass.nextgis.com" + path + "/api/interpreter";
    }
    public static string Load(ImportLog log)
    {
        try { return File.Exists(FilePath) ? Normalize((string?)JObject.Parse(File.ReadAllText(FilePath))["serviceUrl"]) : ""; }
        catch { log.Warn("Service setting unavailable or invalid; public Overpass backups will be used. Configure your service in the import panel."); return ""; }
    }
    public static bool Save(string value, ImportLog log)
    {
        try
        {
            var path = FilePath; Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            var temporary = path + ".tmp";
            File.WriteAllText(temporary, new JObject { ["version"] = 1, ["serviceUrl"] = value }.ToString(Newtonsoft.Json.Formatting.None));
            if (File.Exists(path)) File.Replace(temporary, path, null); else File.Move(temporary, path);
            log.Diagnostic($"OSM SERVICE settingSaved=true; configured={value.Length > 0}; credentialLogged=false");
            return true;
        }
        catch { log.Warn("Could not save the service setting. Check access to the mod settings folder and try again."); return false; }
    }
}
