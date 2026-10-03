# Network hardening status — 2026-10-03

This document records network safeguards, automated coverage and remaining editor verification. Automated tests do not establish full in-game correctness.

## Implemented

- Independent four-tier road selector and rail checkbox; Primary+ / rail enabled defaults, versioned preference file, corrupt-value defaults and legacy secondary migration.
- Selective Overpass network clauses; disabled types omitted from normalization, prefab resolution and placement, with no intentional-absence warnings. Shared forest acquisition remains unchanged.
- Configuration and map/import-generation fences invalidate prepared work. Native load callbacks cancel tasks, clear queues and per-map UI/results, preserve download caches and preferences. Cleanup fences stale work before attempting tracked-definition cleanup; never indiscriminately clears the newly loaded map.
- Conservative coincident-road deduplication, evidence-based divided corridor association, source-geometry fallback for unsupported carriageway crossovers, native direction/width preflight. One-way source roads cannot silently become two-way boulevards. Real shared junctions are not rejected for zero carriageway separation.
- Small aligned bridge-gap reconstruction requires source/corridor/name evidence, compatible class/direction and side alignment; ambiguous or unrelated nearest roads are not connected.
- Short link edges protected before consolidation. Road-only vertical profiles use centerline samples, explicit compatible inclines, common source-junction heights, portal-based structures, local refinement and connected bridge approach rise propagation. Systemic/all-road rejection stops preparation before map replacement.
- Yard simplification restricted to strongly dense tagged service tracks with redundant alternate topology; outside footprint tracks, mainlines and disconnected terminal groups preserved. Native rail fallback prefers narrowest verified train geometry rather than silently selecting a double-track corridor for an individual source track.
- Responsive confirmation layout with native scrollable body and separate footer; shared primary/secondary/utility/destructive button variants and focus/disabled states. Immediate repeat-click guards and backend acknowledgement allow safe retries.
- Selected configuration, repair/skip counters, partial-import warnings and preparation/completion statistics logged.

## Automated coverage

The network regression executable covers all eight road/rail combinations, selective query clauses, missing/corrupt/legacy preferences, native one-way policy, divided/coincident corridors, shared junction width protection, aligned bridge gaps versus perpendicular roads, protected links, centerline length and grade checks, flat/bridge/tunnel profiles, all-road rejection, yard envelope/mainline preservation, cancellation and stale map/configuration epochs.

UI smoke tests exercise independent selections, repeat-click suppression, recoverable preparation retries with logs retained, native confirmation scrolling, Escape/close and disabled application cancellation. Existing processing, Overpass, terrain, conditioning, feature and location-picker tests remain required.

## Remaining limitations / acceptance work

- Full in-game regression matrix has not been run: single/double track appearance, dual decks, interchanges/collector lanes, enormous yards, high-resolution terrain grades, map A → B transitions while native generation is pending, mod reload, save/reload and 1080p/1440p/high-scaling/localization layouts. A build and synthetic tests cannot prove these behaviors.
- Corridor recognition is conservative and local. Curved/diverging unnamed corridors and long unmatched source sections may remain unassociated. Duplicate cleanup removes near-exact representations; it is not a general conflation engine for offset duplicates.
- Horizontal checks target paired carriageway crossings and native widths; exhaustive malformed-junction, acute-turn and wrong-ramp attachment classification is not implemented.
- Vertical fitting ensures compatible positions, but globally continuous grade transitions between independently fitted source chains are not guaranteed. Highway grades above 12% are flagged for source review, not automatically resolved against a second elevation source. Complex branched bridge/tunnel groups can remain unresolved. Peak-grade diagnostics sample derivatives; acceptance additionally uses conservative derivative bounds with subdivision, rejecting uncertain intervals rather than trusting discrete samples alone.
- Rail yards require service tags plus density. Untagged density-only yards are not simplified. Retention is topology/envelope based with a local spacing heuristic, not a verified native-width/catenary envelope model; explicit major-lead/station-throat recognition and multi-track consolidation require further runtime inspection and representative maps.
- Map isolation fences importer tasks and tracked pending definitions. The game's loader remains responsible for unloading already committed old-map entities. Zero stale live entities must still be verified in the editor; no broad new-map cleanup is used to conceal a lifecycle issue.

No changes to terrain generation, hydrology, climate, vegetation generation, surface classification or air-connection behavior are included in this follow-up. Earlier guarded/unimplemented optional stages remain as documented in FEATURE_STATUS.md.
