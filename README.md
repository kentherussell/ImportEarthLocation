# Import Earth Location

Pick a place. Bring it into Cities: Skylines II.

The goal is simple: take any location in the world and turn it into a realistic starting map, with terrain, roads and rail that reflect the real place—not just a flat map with roads drawn on it.

## How to use it

1. Open a map in the Map Editor and choose a location or enter coordinates.
2. Select your map size, road level and whether to include rail.
3. Prepare the import, review the results, then confirm replacement.

Save a backup first: importing replaces the current map contents.

## Current status

A work in progress targeting Cities: Skylines II 1.6.2f1. This is a map importer, not a complete city recreation; buildings and zoning aren't included. Inland water creation is not yet supported.

See [feature status](docs/FEATURE_STATUS.md) for current limitations and testing notes.

## Local verification

With PowerShell 7, .NET 9, Node.js, the UI dependencies and the Cities: Skylines II modding toolchain installed:

```powershell
pwsh -NoProfile -File tests/verify.ps1 -Focus Full
pwsh -NoProfile -File tests/verify.ps1 -Focus Overpass
```

The full gate builds each test harness once, then runs all offline regression checks with bounded parallelism. It also compiles against the installed game libraries and stages the UI bundle locally; it does not deploy, launch the game or contact live data providers. Targeted modes are faster feedback, not substitutes for the full gate. Timings and individual logs are saved under `bin/test-results`. For captured native-batch replays, options and coverage limits, see [test workflow notes](docs/TEST_OPTIMIZATION_2026-10-09.md).

## License

MIT. Uses public elevation and OpenStreetMap data; see [third-party attribution](THIRD_PARTY.md).
