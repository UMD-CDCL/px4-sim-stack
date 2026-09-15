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

The packet entry points are `verify/code-test.sh`, `verify/bench-test.sh`,
`verify/gps-test.sh`, and `verify/sim-test.sh`. They are dry-run by default:
they record the selected existing stages and repository checkpoint under
`verify/evidence/<stage>/`, then report `PENDING`. Use `--live` only when the
required stack is already running; live mode never starts, stops, or resets
services. A live `PASS` is based solely on the exit status and raw output of
the selected stage runner, while a dry run is never evidence of a pass.

The wrappers select these existing checks: code (`airframes contract units`),
bench (`ground foxglove`), GPS (`localize ground`), and simulation
(`vehicle flight captures foxglove`). The existing stage files remain the
source of runtime assertions; the wrappers only provide isolation, checkpoint
metadata, and retained reports.

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

## Fast classification

Use one intent tag and, independently, a tested marker for every feature
record. `CURRENT` describes the intended baseline contract, not proof that the
feature works.

| Tag | Meaning | How to decide quickly |
|---|---|---|
| `CURRENT` | Intended to be part of the current baseline contract | The feature belongs in the current user-facing system, whether or not it is implemented yet |
| `ORPHANED` | Described or configured, but no current code path reaches it | The old doc/config exists, but search finds no active launch, publisher, service, or caller |
| `MISSING` | Intended in the current contract, but implementation or wiring is absent | The feature is required now, but no implementation can be found |
| `PLANNED` | Intentionally deferred future capability that may affect later design or status reports | Approved or proposed for a later increment, outside the current baseline contract |
| `BLOCKED` | Implemented, but the current machine cannot exercise it | Code and wiring exist; record the concrete environmental blocker |
| `UNKNOWN` | Evidence is insufficient or sources disagree | Do not infer; record the conflicting files or question for review |

`TESTED` is independent of intent. Write `Tested: YES` only when the feature
passes its stage assertion at a recorded checkpoint. Use `Tested: NO` when it
has not been exercised and `Tested: BLOCKED` when the environment prevents the
test. A file existing is not enough for `TESTED`.

The shortcut is: **intended now means `CURRENT`; dead description means
`ORPHANED`; intended now but absent means `MISSING`; intentionally deferred
means `PLANNED`; verified at a dated checkpoint means `Tested: YES`.**

## Exhaustive records

Each stage document must record every feature it checks using the same fields:

```text
Intent: CURRENT | ORPHANED | MISSING | PLANNED | UNKNOWN
Tested: YES | NO | BLOCKED
Feature:
User interaction/front door:
Code/config evidence:
Runtime assertion:
Expected result:
Observed result:
Date:
Git status and commits:
Test evidence and checkpoint:
Notes or reviewer question:
```

The stage documents are intentionally separate so a bench test can be run and
reviewed without claiming that GPS or flight behavior was tested.

## Uncertainty policy

Every assertion must be confirmed against code or observed protocol data. If
the implementation does not expose a clear contract, the test report must say
`UNKNOWN`, identify the source inspected, and state what evidence or operator
decision is needed. In particular, the optional ReID model is currently
referenced by launch/runtime code but was not present in the verified image;
the baseline must distinguish that warning from a failed core pipeline.

Planned features belong in the ledger even without implementation. Record
their proposed interaction, owner, front door, dependencies, and checkpoint.
They are not current-baseline failures, but must appear in status reports so
later work does not accidentally make their design impossible. At the stated
checkpoint, a planned feature either remains `PLANNED` or becomes `MISSING` if
it is now part of the required contract.

## Stage matrix

The initial function list and evidence are in [FUNCTION-MATRIX.md](FUNCTION-MATRIX.md).
The exhaustive feature inventory starts in [FEATURE-LEDGER.md](FEATURE-LEDGER.md),
with broader package inventories in [MAVINSIGHT-LEDGER.md](MAVINSIGHT-LEDGER.md),
[PX4SIM-LEDGER.md](PX4SIM-LEDGER.md), and [5G-DRONE-LEDGER.md](5G-DRONE-LEDGER.md),
plus the deployment inventory in [CHIMERA-DEPLOY-LEDGER.md](CHIMERA-DEPLOY-LEDGER.md),
and the stage record templates are [CODE-TEST.md](CODE-TEST.md),
[BENCH-TEST.md](BENCH-TEST.md), [GPS-TEST.md](GPS-TEST.md), and
[SIM-TEST.md](SIM-TEST.md). Please edit the ledger classifications and expected
results before implementation of the stage scripts.
