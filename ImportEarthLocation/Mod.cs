using Colossal.Logging;
using Game;
using Game.Modding;
using ImportEarthLocation.Systems;

namespace ImportEarthLocation;

public sealed class Mod : IMod
{
    public const string Id = "ImportEarthLocation";
    public const string Version = "0.1.0";

    public static readonly ILog Log = LogManager
        .GetLogger($"{Id}.{nameof(Mod)}")
        .SetShowsErrorsInUI(false);

    public void OnLoad(UpdateSystem updateSystem)
    {
        Log.Info($"Loading {Id} {Version}");
        Core.ImportSession.ChangeMap(Game.SceneFlow.GameManager.instance.gameMode == GameMode.Editor && !Game.SceneFlow.GameManager.instance.isGameLoading);
        updateSystem.UpdateAt<ImportEarthLocationUISystem>(SystemUpdatePhase.UIUpdate);
        updateSystem.UpdateAt<ImportApplySystem>(SystemUpdatePhase.PreTool);
        updateSystem.UpdateAt<NetworkCommitSystem>(SystemUpdatePhase.ToolUpdate);
    }

    public void OnDispose()
    {
        Core.ImportSession.ChangeMap(false);
        ImportApplySystem.AbortPending();
        NetworkCommitSystem.CancelPending();
        Log.Info($"Disposing {Id}");
    }
}
