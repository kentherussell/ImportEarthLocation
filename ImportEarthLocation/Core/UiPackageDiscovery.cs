using System;
using System.IO;

namespace ImportEarthLocation.Core;

// Byte-loaded assemblies have no filesystem Location under the game's Mono
// loader. Discovery is optional and must never prevent backend initialization.
public static class UiPackageDiscovery
{
    public static string? Find(string moduleId,Func<string?> executablePath,Func<string?> assemblyPath,
        Func<string?> localDirectory,Action<string>? diagnostic=null)
    {
        string? Attempt(string origin,Func<string?> getPath,bool directory)
        {
            try
            {
                var path=getPath();
                if(string.IsNullOrWhiteSpace(path)){diagnostic?.Invoke($"UI discovery {origin}: no filesystem path; trying next source");return null;}
                if(Uri.TryCreate(path,UriKind.Absolute,out var uri))
                {
                    if(!uri.IsFile){diagnostic?.Invoke($"UI discovery {origin}: non-file URI; trying next source");return null;}
                    path=uri.LocalPath;
                }
                if(path!.IndexOf('\0')>=0||!Path.IsPathRooted(path))return null;
                var folder=directory?Path.GetFullPath(path):Path.GetDirectoryName(path);
                if(string.IsNullOrWhiteSpace(folder)||ModuleForDirectory(folder,moduleId)==null)return null;
                diagnostic?.Invoke($"UI discovery {origin}: complete package found at {folder}");
                return folder;
            }
            catch(Exception error)
            {
                diagnostic?.Invoke($"UI discovery {origin}: deferred ({error.GetType().Name}: {error.GetBaseException().Message}); trying next source");
                return null;
            }
        }
        return Attempt("executable-asset",executablePath,false)??Attempt("assembly",assemblyPath,false)??Attempt("local-mod",localDirectory,true);
    }

    public static string? ModuleForDirectory(string? directory,string moduleId)
    {
        if(string.IsNullOrWhiteSpace(directory))return null;
        if(!File.Exists(Path.Combine(directory!,moduleId+".css")))return null;
        return File.Exists(Path.Combine(directory!,moduleId+".mjs"))?moduleId+".mjs":
            File.Exists(Path.Combine(directory!,moduleId+".js"))?moduleId+".js":null;
    }
}
