# GPS test

## Scope

Verify only functions requiring a valid fix: MAVROS global/local position,
datum and geoid handling, TF/map placement, localization, target geometry,
scoring, mosaic, fiducial operations, and GPS-dependent Foxglove panels.

## Isolation

This stage starts from its own clean setup and records the injected or
simulated fix, datum, tolerances, and teardown. Bench evidence may identify
prerequisites, but cannot substitute for GPS-stage evidence.
