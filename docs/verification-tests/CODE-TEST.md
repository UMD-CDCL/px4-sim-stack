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
