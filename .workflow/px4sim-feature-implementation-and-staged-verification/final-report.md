# Final Report: px4sim feature implementation and staged verification

## Outcome
Implementation checkpoint complete on `feature/px4sim-baseline-implementation`.

Start tag: `ubuntu24-compat-start-20260915` at `3188ce1`.
Implementation commit: `5edc0ad` (`implement px4sim front door and staged verification`).

## Accepted Results
The px4sim front door now treats `restart` as an offline prepared-image restart and supports `restart --build` for cache-preserving rebuilds. The TUI action text and restart feed reset match that contract.

The code, bench, GPS, and sim verification wrappers are separate, dry-run by default, checkpointed, and capable of invoking existing stage assertions in live mode without lifecycle side effects.

## Rejected Results
The MAVInsight parser change was rejected because it exists only in the ignored generated `.build-contexts` tree and is not portable in a clean checkout.

## Conflicts Resolved

## Verification Evidence
Passed: `./px4sim check`; `./px4sim help`; Python compilation; shell syntax checks; all four wrapper dry runs; workflow artifact validation; `git diff --check`; simulator range-reading unit tests; startup/frame contract checks.
The code stage passed 17/17 assertions at `dd42eca`. The bench stage remains blocked by TF/frame parity and startup resource timing. A cache-aware rebuild reached cached layers but was blocked by DNS resolution of Ubuntu package mirrors; no new runtime evidence is claimed from that build.

## Remaining Risks
The runtime feature packet remains to be reworked against tracked source. Existing fixture coverage still reports a missing `/src/tracking_test_5g` path, and `pytest` was unavailable for the rejected packet. The rebuilt images containing the new frame/readiness changes still need a successful networked build and restart before live bench rerun.

QGroundControl launch serialization was added in `c5bf7c5`: a persistent
config-volume flock plus a 30-second stop grace period. The current running
stack has exactly one QGroundControl process. Rebuilding that image to exercise
the entrypoint is currently blocked by package-mirror DNS, so runtime
single-instance behavior remains pending direct evidence.

## Reusable Follow-up
