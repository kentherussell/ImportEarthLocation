# Import pipeline hardening

The import pipeline validates source data and native integration before destructive
application. Coordinates, map size, preparation and confirmation define the workflow.

## Processing and safety

- OSM retains 16 geographic tiles. Three rolling workers run at most three HTTP requests;
  HTTP 429 lowers new request concurrency from 3 to 2 to 1. No hedged/duplicate tile requests.
- Retry budgets remain six transient failures per provider, with provider cooldowns and
  bounded backoff. Permanent HTTP failures abort. Retry-After instructions beyond five
  minutes stop preparation rather than waiting without a useful bound.
- Indexed tile results make parsing independent of download completion order. Successful
  tiles are cached immediately. Query version, bounds, tile index and secondary-road setting
  are part of the cache identity. The forest query uses cache version v3; older road-only
  OSM cache entries cannot provide vegetation coverage and are not reused. Terrain cache is
  unchanged. Incomplete geographic coverage aborts preparation, preserving successful cache.
- Parsing skips malformed individual features, but invalid whole documents/server error
  remarks abort preparation. CPU terrain preparation is limited to two parallel workers;
  the XML parser uses one additional worker. Normalization and vegetation preparation run
  on background continuations with cancellation checks.
- Network normalization protects source topology, way endpoints, tag/direction/layer
  boundaries, clipping points and sharp bends. It simplifies only unprotected shapes,
  fits cubic sections and splits failed fits until normalized source lines can be used.
  Horizontal source-deviation limits are 2 m for roads and 0.75 m for rail, including
  conservative between-sample error allowances. Junction positions do not move.
- Duplicate removal requires compatible network attributes and shared topology endpoints;
  post-normalization near-duplicates additionally require nearly coincident curves.
  Different-ID parallel tracks/carriageways and ambiguous overlaps are preserved.
- Coincident endpoints with different topology identities explicitly disable native merge.
  Crossing geometry alone does not introduce a source junction. Bridge/tunnel elevation is
  passed explicitly to the native course, rather than silently converted to a surface road.
- Ordinary OSM railway ways default to single-track prefab preference; supported `tracks`
  tags select double-track preference. Rail fallback never uses road prefabs.
- Forest masks support closed wood/forest ways and stitched relation outer/inner rings.
  Open, incomplete or ambiguous rings are skipped. Candidates are deterministic jittered
  points with 12 m minimum spacing, map/water/corridor exclusions, density reduction and
  a hard 75,000-tree cap. Placement is batched in 512-candidate native definition batches.
  Missing tree prefabs and preparation/placement failures are non-fatal.
- `WaterLevels.Effective(nominal)` is the only source for the +1 m correction. Encoding
  remains unchanged; both tree water exclusion and water application use the effective level.
- Validation and explicit confirmation precede entity deletion, terrain replacement and
  native placement. Cancellation is disabled during destructive application. No rollback
  mechanism was added. Most-network-section failures report a major import failure.
- Completion logs include cache/retry counts, normalization/duplicate/merge counts, road/rail
  committed section counts, skipped sections, tree committed definition counts and timings.
  Committed definition counts are not advertised as a count of final ECS sub-entities.

## Reproducible offline checks

From the repository root, with .NET SDK 9 installed:

```powershell
dotnet restore tests/overpass-tests/overpass-tests.csproj --configfile tests/overpass-tests/NuGet.Config
dotnet run --project tests/overpass-tests/overpass-tests.csproj --no-restore
dotnet restore tests/processing-tests/processing-tests.csproj --configfile tests/processing-tests/NuGet.Config
dotnet run --project tests/processing-tests/processing-tests.csproj --no-restore
dotnet restore tests/terrain-tests/terrain-tests.csproj --configfile tests/terrain-tests/NuGet.Config
dotnet run --project tests/terrain-tests/terrain-tests.csproj --no-restore
node tests/ui-smoke.cjs
```

Processing checks use the installed Unity math assembly. If the game is installed elsewhere,
pass `-p:GameManagedPath=".../Cities2_Data/Managed"`. HTTP tests simulate responses, not public
server availability. UI and terrain API checks use stand-ins for game rendering/native calls.
They do not establish in-game acceptance.

## Required in-game acceptance run

Use a saved backup or disposable editor map. Verify a coastal location with rail and forest:

1. Prepare/cancel during downloads and processing. Existing map contents must remain unchanged.
2. Retry preparation: successful v3 OSM/terrain tiles should be reused and logs remain visible.
3. Before confirmation, inspect the summary; no map entities may have changed.
4. Confirm replacement; inspect straight/gentle curves, junctions, sharp bends, parallel rail,
   crossings, bridges, tunnels and clipped boundary endpoints. Verify native topology visually.
5. Confirm water is nominal encoded sea level plus exactly 1 m. Inspect forests, polygon holes,
   shoreline/network exclusions, tree rendering, and save/reload persistence.
6. Inspect detailed completion metrics and any optional-vegetation warnings.

The acceptance criteria are not fully verified until this in-game run passes. In particular,
native curve/topology/elevation handling and native tree generation cannot be proven by
offline component tests or compilation alone.
