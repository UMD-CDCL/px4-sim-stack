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
Date: 2026-09-15. Commit: `dd42eca`. Branch: `feature/px4sim-baseline-implementation`.
Git status at test start: clean. Evidence: external checkpoint report under
`/tmp/px4sim-verification/code/`; this does not certify runtime bench, GPS, or
flight behavior.
