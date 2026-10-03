# Implementation and verification status — 2026-10-03

The mod is under development. This document records implemented features, limitations and verification requirements for Cities: Skylines II 1.6.2f1. Native editor operation and save/reload require manual testing.

## Implemented paths

- Location picker: search with bounded fallback, visible cached map tiles, pan/zoom/click, fixed reticle and map-size footprint, draft coordinates, explicit commit/cancel, stale-response protection and manual fallback. MapExt choices require its readable current-size API. Picking does not prepare or apply an import.
- Ground surfaces (off initially): explicit OSM sand/beach, grass/meadow/grassland and quarry dirt polygons, holes, deterministic priorities, geographic feathering and exclusive intensity masks. Native paint channels/textures are probed before application; incompatible native formats/material mappings skip the stage. Unknown cover stays base. Dry-grass inference, climate fallback, low-confidence coastline inference and insignificant-patch merging are not implemented.
- Air connections (off initially): bundled Natural Earth city metadata, deterministic nearest-first eligibility, great-circle bearings, actual native airway-grid perimeter, minimum spacing including corners, congestion reduction and named native prefab-archetype objects. Required native initialization components are verified. Actual flight routing and serialization still require in-game acceptance tests.
- Climate: validated/cached NASA POWER monthly normals, numerical seasonal comparison against native climate prefabs, bounded altitude adjustment and meteorological FROM-to-flow wind conversion. Shared climate assets are not mutated. Existing native wind strength/pressure are retained because a physical-speed conversion is unverified. First use requires network access; a global offline normals dataset and cloud-normal scoring are not implemented.
- Terrain conditioning: immutable source mosaic plus working buffer; complete tile/dimension/finite/range checks; bounded slope-following NoData repair; protected mapped coast/cliff/water/quarry envelopes; isolated artifact cleanup and light residual-noise reduction; ambiguous seam detection without smoothing real cliffs; numerical/change-budget validation with optional-filter rollback; bilinear resampling without ringing or artificial detail; encoding rejects invalid values instead of clamping. Source and correction metrics appear in logs and preparation summary.

## Deliberately stopped or outstanding stages

- Inland water creation is NOT implemented. CS2 1.6's active editor water definition path no longer uses the legacy constant-depth field. Native source level/flow semantics have not been established safely. Rivers/lakes/reservoirs are classified for diagnostics only; no guessed sources, flow multiplication, water leveling or canyon carving is performed. Global native sea-level assignment remains in the existing terrain bridge; actual coastal/ocean coverage still needs native verification. A coastal map without working sea coverage is not a passed water acceptance case.
- Optional water/surface geography uses separate cached queries with the existing provider rotation/retry behavior, bounded by a three-minute overall deadline so an unavailable optional service cannot indefinitely delay a valid core import. Failed optional geography skips dependent classification/painting; terrain/road/rail download retries remain unchanged.
- The separate bounded hydrology correction interface validates a verified corridor/protected rim and <=3m lowering against the immutable source, then applies proposals atomically. It is not invoked without a verified hydrology plan. River-width blending, directed tributary/lake water planning and flood preflight remain outstanding.
- Ambiguous seams are preserved. Confident narrow seam correction/source-fallback decisions require corroborating source evidence; large coherent steps cannot be distinguished from real cliffs using a tile boundary alone. Multi-cell artifact cleanup, geography-backed coastal correction and a comprehensive post-resample slope/artifact audit remain outstanding.
- Native surface application and climate assignment have current-assembly API checks, but renderer behavior, named air object initialization, native serialization and functional flights require a disposable editor map and save/reload testing.

## Automated checks

Run from repository root with .NET 9 and the installed game assemblies:

```powershell
dotnet run --project tests/conditioning-tests
dotnet run --project tests/feature-tests
dotnet run --project tests/processing-tests
dotnet run --project tests/overpass-tests
dotnet run --project tests/terrain-tests
dotnet run --project tests/network-tests
node tests/ui-smoke.cjs
node tests/picker-smoke.cjs
```

The conditioning suite covers flat/sloped terrain, isolated spikes/pits/noise, protected summits/quarries, coasts/islands, real tile-aligned cliffs, ridges/basins, small/large/ocean NoData, impossible/non-finite elevations, cancellation, optional-filter rollback and atomic river-corridor/rim limits. The feature suite covers deterministic offline destinations, perimeter/corner spacing, wind-vector conventions and surface priority/holes/base/water/feathering. The picker suite checks coordinate/projection math, partial drafts, explicit commit/cancel, no import side effects, stale/reopened responses and failed preview fallback. These are planner/mocked UI tests, not native game acceptance tests.

## Required editor checks before claiming completion

Restart CS2 to load both rebuilt mod halves. Use a disposable map and save a backup. Verify picker pan/click/zoom versus manually entered coordinates; closed picker retains committed coordinates; logs remain fixed-height and scrollable across retries; cancelling preparation leaves terrain untouched. Test explicit surface cover and holes, repainting/save/reload, names and functional air outside connections/save/reload, chosen native climate/weather and wind/save/reload. Repeat on vanilla and a verified MapExt map. Review skipped-stage messages: a successful core import is not a successful inland-water import. Validate coastal cliffs, archipelagos, narrow landforms, mountains, low-gradient rivers, lake rims and extreme elevations against source geography before relaxing conservative terrain protections.
