# Target runtime snapshot

Checked on 2026-10-02.

- Current game line targeted: Cities: Skylines II **1.6.2f1**.
- Latest tracked stock code/UI template change: `CitiesSkylinesModding/StockModTemplatesDiffer` commit `1d7c741d14e61c6e2e55b720d61f39a85a56f87f`, identified by that repository as the 1.6.0f1 template update. No later stock-template structure change was present.
- Current MapExt behavior/detection reference: `Noel-leoN/MapExt2` commit `949272f34a9276eab1fce046a4bff4022961c6fb` (2026-09-15). The MVP intentionally exposes only the non-experimental 28.672 km and 57.344 km modes in addition to vanilla; it does not expose MapExt's experimental 43 km mode or the 114 km mode.
- Current native network-placement reference: `iRecycled/CS2_Events` commit `b196491a7f906042d5e19c075e8238fd30f67f6a`, which documents and implements the two-frame `CreationDefinition + NetCourse` path used by CS2's normal `GenerateNodes/GenerateEdges` and `ApplyNetSystem` phases.
- Current prefab/editor integration cross-check: `JadHajjar/RoadBuilder-CSII` commit `b9fbd0f84400b43e0cfc405c1ace0cdf73502521`.

The source deliberately probes the two unstable internal seams before confirmation:

1. `TerrainSystem.ReplaceHeightmap(Texture2D)` plus the runtime terrain scale/offset hook.
2. The tool pipeline's native apply-mode hook after `GenerateEdgesSystem` and before `ToolOutputSystem`.

If either seam changes in a later game update, preparation fails before deleting or replacing map contents. Re-check those two integration classes against the installed `Game.dll` when moving beyond 1.6.2f1.
