# Verification tests

These tests verify the portable operator path from the 5G Drone Foxglove
configuration, through `chimera_sim.json`, into the running system. The
controller and QGroundControl are outside this scope. `px4sim` and Foxglove
are front doors and are tested as such.

The test stages are deliberately separate and sequestered:

1. `code-test.sh` checks that source, launch files, configuration, models,
   services, topics, and front-door commands exist before running the stack.
2. `bench-test.sh` checks ground-only behavior with GPS-dependent behavior
   disabled or withheld.
3. `gps-test.sh` checks position, datum, map, localization, and other
   functions that require GPS or a valid geodetic fix.
4. `sim-test.sh` checks the complete in-flight simulation, including PX4,
   MAVLink, cameras, perception, mission, and operator outputs.

Each stage will have its own executable test file, evidence directory, and
report. A stage must not silently depend on another stage's running processes;
its setup and teardown belong in that stage's file. The tests should use the
same configuration and service front doors that an operator uses, with
hardware-specific seams isolated so the procedure can later be adapted to a
real aircraft.

## Baseline rule

The baseline is recorded at a critical checkpoint, not on every code change.
At the start and end of a completed cycle, record the date, repository commit,
branch, and clean/dirty status. The tested state is updated only after the
stage has completed and its evidence has been reviewed.

The current baseline is **2026-09-14**, after the `sim running` checkpoint.
The working tree was intentionally dirty because compatibility changes are
being staged for review. All modified repositories were on
`feature/ubuntu24-compat`; the root and `chimera-deploy` branches were pushed.
The PX4 mirror commit was created on that branch but could not be pushed from
this machine because GitHub HTTPS credentials were unavailable.

This verification suite is a critical-checkpoint tool. It does not need to be
run often or after every edit. Its purpose is to establish a trustworthy
baseline before substantial changes, then detect regressions at the next
checkpoint.

## Uncertainty policy

Every assertion must be confirmed against code or observed protocol data. If
the implementation does not expose a clear contract, the test report must say
`UNKNOWN`, identify the source inspected, and state what evidence or operator
decision is needed. In particular, the optional ReID model is currently
referenced by launch/runtime code but was not present in the verified image;
the baseline must distinguish that warning from a failed core pipeline.

## Stage matrix

The proposed functions and evidence are in [FUNCTION-MATRIX.md](FUNCTION-MATRIX.md).
Please edit that table before implementation of the stage scripts.
