# Code test

## Scope

Static completeness only: source, package metadata, launch files, parameters,
models, Foxglove layout, `px4sim` commands, Compose profiles, and test hooks.
No running system is evidence for this stage.

## Exhaustive feature records

Record each row from [FEATURE-LEDGER.md](FEATURE-LEDGER.md), plus every launch
node, service, publisher, subscriber, model, and front-door command found by
tracing the source. Use the record format in the README. A static-only finding
must not be marked `Tested: YES` until its runtime stage passes. `CURRENT` only
records intended scope. Record planned
features and their intended ownership even when no source exists yet.

## Checkpoint

Tested: YES for the executed code-stage assertions (17 passed, 0 failed).
Date: 2026-09-15. Commit: `373fee7`. Branch: `feature/px4sim-baseline-implementation`.
Git status at test start: dirty only from retained generated evidence. Evidence
is `verify/evidence/code/`; this does not certify runtime bench, GPS, or flight
behavior.

Latest tested checkpoint: **19 passed, 0 failed** on 2026-09-16 at commit
`09175a5`, including the PX4SIM UI autocomplete/output-wrapping
regression checks, the complete functional 5G Drone suite (133 passed), and
packaged terrain/MAVInsight tests. The test runner now quotes pytest arguments
when invoking the onboard container. QGC is explicitly front-door owned and
does not auto-restart after its singleton guard rejects a duplicate launch.
After `./px4sim restart --no-build`, all services reached ready state and the
QGC container had exactly one `/opt/qgc/usr/bin/QGroundControl` process; its
logs showed normal initialization and no second-instance error.

The same code packet also covers the TUI stack-menu cancellation action, which
uses the existing process-group cleanup path; the focused helper packet is
five tests and the full code gate remains **19 passed, 0 failed**. Service
rows now also expose one normalized `lifecycle` value (`not_started`,
`starting`, `running`, `failed`, `stopped`, or `unknown`) so px4sim UI and
other front-door consumers do not independently infer initialization state.

The live code gate was rerun at `09175a5` with the prepared images and again
returned **19 passed, 0 failed**. The simulator Dockerfile and camera
supervisor changes are source-checked here; their rebuilt-image recovery test
remains pending the BuildKit apt/DNS issue recorded in the bench ledger.
