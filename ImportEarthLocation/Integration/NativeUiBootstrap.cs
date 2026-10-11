using System;
using System.IO;
using Game.SceneFlow;

namespace ImportEarthLocation.Integration;

// Use the same public host/binding APIs as ModManager.AddUIModule. A local DLL
// can load before its JS asset is discovered; do not depend on another mod's
// subsequent UI refresh to make this module available.
internal static class NativeUiBootstrap
{
    private static string? s_Directory;
    private static Game.Modding.IMod? s_Mod;
    private static Game.UI.AppBindings? s_RegisteredBindings;
    private static Colossal.UI.UISystem? s_RegisteredHost;
    private static string? s_RegisteredDirectory;
    private static string? s_RegisteredModule;
    private static DateTime s_NextAttempt;
    private static bool s_ReportedMissing;
    public static void Initialize(Game.Modding.IMod mod)
    {
        Reset();s_Mod=mod;s_ReportedMissing=false;
        try
        {
            s_Directory=Discover(mod);
            Mod.Log.Info($"UI bootstrap backend loaded; directory={s_Directory??"deferred"}; initialized={GameManager.instance.modManager.isInitialized}; build={typeof(Mod).Assembly.ManifestModule.ModuleVersionId}; discoveryCannotAbortBackend=true");
        }
        catch(Exception error){Mod.Log.Warn("UI package discovery deferred; backend initialization continues: "+error.GetBaseException().Message);}
    }

    private static string? Discover(Game.Modding.IMod mod)=>Core.UiPackageDiscovery.Find(Mod.Id,
        ()=>GameManager.instance.modManager.TryGetExecutableAsset(mod,out var asset)?asset.path:null,
        ()=>typeof(Mod).Assembly.Location,
        ()=>Path.Combine(UnityEngine.Application.persistentDataPath,"Mods",Mod.Id),
        message=>Mod.Log.Info(message));

    public static void Ensure()
    {
        if(DateTime.UtcNow<s_NextAttempt)return;
        s_NextAttempt=DateTime.UtcNow.AddSeconds(5);
        try
        {
            var game=GameManager.instance;
            if(!game.modManager.isInitialized||game.userInterface?.appBindings==null||Colossal.UI.UIManager.defaultUISystem==null)return;
            var bindings=game.userInterface.appBindings;
            var host=Colossal.UI.UIManager.defaultUISystem;
            if(ReferenceEquals(s_RegisteredBindings,bindings)&&ReferenceEquals(s_RegisteredHost,host))return;
            DetachRegistration();
            if(Core.UiPackageDiscovery.ModuleForDirectory(s_Directory,Mod.Id)==null&&s_Mod!=null)s_Directory=Discover(s_Mod);
            var module=Core.UiPackageDiscovery.ModuleForDirectory(s_Directory,Mod.Id);
            if(module==null)
            {
                if(!s_ReportedMissing){Mod.Log.Warn($"UI bootstrap package incomplete: JS/CSS missing from backend directory {s_Directory}; button cannot load independently.");s_ReportedMissing=true;}
                return;
            }
            // AddActiveUIModLocation is a HashSet-backed, idempotent operation in
            // the installed engine. Never reinitialize every mod's UI manager.
            // Track resources before calls so a partial registration can also
            // be detached if the native UI call throws or the host is replaced.
            s_RegisteredHost=host;s_RegisteredDirectory=s_Directory;
            host.AddHostLocation("ui-mods",s_Directory,true,0);
            s_RegisteredModule="coui://ui-mods/"+module;s_RegisteredBindings=bindings;
            bindings.AddActiveUIModLocation(new[]{s_RegisteredModule});
            Mod.Log.Info($"UI bootstrap module available: coui://ui-mods/{module}; standalone registration=true; frontend acknowledgement still required");
        }
        catch(Exception error){DetachRegistration();Mod.Log.Warn("UI bootstrap deferred: "+error.GetBaseException().Message);}
    }

    public static void Frontend(string state)
    {
        // The UI is untrusted diagnostic input, not executable content.
        Mod.Log.Info("UI frontend "+(state.Length>1500?state.Substring(0,1500):state));
    }
    public static void Reset()
    {
        DetachRegistration();
        s_Directory=null;s_Mod=null;s_NextAttempt=DateTime.MinValue;
    }

    private static void DetachRegistration()
    {
        // Remove only this package's path and URL, never the shared ui-mods host.
        // UI teardown may precede mod disposal; cleanup must not abort disposal.
        try{if(s_RegisteredBindings!=null&&s_RegisteredModule!=null)s_RegisteredBindings.RemoveActiveUIModLocation(new[]{s_RegisteredModule});}
        catch(Exception error){Mod.Log.Warn("UI module cleanup deferred: "+error.GetBaseException().Message);}
        try{if(s_RegisteredHost!=null&&s_RegisteredDirectory!=null)s_RegisteredHost.RemoveHostLocation("ui-mods",s_RegisteredDirectory);}
        catch(Exception error){Mod.Log.Warn("UI host cleanup deferred: "+error.GetBaseException().Message);}
        s_RegisteredBindings=null;s_RegisteredHost=null;s_RegisteredModule=null;s_RegisteredDirectory=null;
    }
}
