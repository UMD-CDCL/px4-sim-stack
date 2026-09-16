# GPS test

## Scope

Verify only functions requiring a valid fix: MAVROS global/local position,
datum and geoid handling, TF/map placement, localization, target geometry,
scoring, mosaic, fiducial operations, and GPS-dependent Foxglove panels.

## Isolation

This stage starts from its own clean setup and records the injected or
simulated fix, datum, tolerances, and teardown. Bench evidence may identify
prerequisites, but cannot substitute for GPS-stage evidence. Future GPS
capabilities remain `PLANNED` until their checkpoint makes them required.

## Tested checkpoint

On 2026-09-16, the live GPS packet at root commit `5d6a275` completed with
**21 passed, 0 failed**. It verified localization geometry at multiple camera
framings and attitudes, terrain and roof intersections, horizon gating,
ground-station forwarding, scoring, and click behavior. Evidence is in
`verify/evidence/gps/`; metadata records the tested worktree state.
