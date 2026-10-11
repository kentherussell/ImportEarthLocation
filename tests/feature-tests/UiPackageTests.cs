using ImportEarthLocation.Core;

internal static class UiPackageTests
{
    public static void Run()
    {
        void Check(bool ok,string message){if(!ok)throw new Exception(message);}
        var temp=Path.GetFullPath(Path.GetTempPath());
        var root=Path.GetFullPath(Path.Combine(temp,"iel-ui-package-"+Guid.NewGuid().ToString("N")));
        const string id="ImportEarthLocation";
        try
        {
            var asset=Path.Combine(root,"asset");var local=Path.Combine(root,"local");
            foreach(var folder in new[]{asset,local})
            {
                Directory.CreateDirectory(folder);
                File.WriteAllText(Path.Combine(folder,id+".mjs"),"export default () => {};");
                File.WriteAllText(Path.Combine(folder,id+".css"),"/* package fixture */");
            }
            var notes=new List<string>();
            var byteLoaded=System.Reflection.Assembly.Load(File.ReadAllBytes(typeof(UiPackageTests).Assembly.Location));
            Check(string.IsNullOrEmpty(byteLoaded.Location),"Fixture must reproduce memory-loaded assembly with no filesystem location");
            var selected=UiPackageDiscovery.Find(id,()=>null,()=>byteLoaded.Location,()=>local,notes.Add);
            Check(selected==local&&notes.Any(n=>n.Contains("assembly: no filesystem path")),"Empty byte-loaded assembly path aborted discovery instead of using local package");
            selected=UiPackageDiscovery.Find(id,()=>Path.Combine(asset,id+".dll"),()=>throw new Exception("must not access assembly fallback"),()=>local);
            Check(selected==asset,"Game executable asset did not take priority over assembly/local fallback");
            selected=UiPackageDiscovery.Find(id,()=>throw new ArgumentException("Asset registry not ready"),()=>"\0",()=>local);
            Check(selected==local,"Unavailable asset registry or malformed assembly path interrupted local fallback");
            selected=UiPackageDiscovery.Find(id,()=>"coui://ui-mods/ImportEarthLocation",()=>"relative.dll",()=>local);
            Check(selected==local,"Virtual/relative path treated as physical package");
            selected=UiPackageDiscovery.Find(id,()=>new Uri(Path.Combine(asset,id+".dll")).AbsoluteUri,()=>null,()=>local);
            Check(selected==asset,"Valid executable file URI not resolved");
            Check(UiPackageDiscovery.Find(id,()=>null,()=>"",()=>null)==null,"Unavailable package did not defer cleanly");
            File.WriteAllText(Path.Combine(local,id+".js"),"export default () => {};");
            Check(UiPackageDiscovery.ModuleForDirectory(local,id)==id+".mjs","Webpack module should take priority over legacy JS");
            File.Delete(Path.Combine(local,id+".mjs"));
            Check(UiPackageDiscovery.ModuleForDirectory(local,id)==id+".js","Complete legacy JS fallback missing");
            File.Delete(Path.Combine(local,id+".css"));
            Check(UiPackageDiscovery.ModuleForDirectory(local,id)==null,"Incomplete package accepted without CSS");
            Console.WriteLine("PASS: real byte-loaded assembly empty Location, asset-first discovery, invalid/virtual paths, delayed registry, local fallback, complete mjs/js/CSS package selection.");
        }
        finally
        {
            if(!root.StartsWith(temp.TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))
                throw new Exception("Refusing cleanup outside test temporary directory");
            if(Directory.Exists(root))Directory.Delete(root,true);
        }
    }
}
