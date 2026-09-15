# px4sim feature implementation and staged verification

## Goal
Implement the highest-confidence features enumerated in the ledgers, beginning with easy-to-verify front-door behavior, then validate in staged code, bench, GPS, and flight-simulation tests.

## Success Criteria
Each accepted slice is source-grounded, reviewed, tested at its minimum component level, committed, and pushed on a feature branch derived from the tagged Ubuntu 24 baseline. No work is merged into the compatibility baseline until the slice passes its stated checks.

## Current Context
Baseline: `feature/ubuntu24-compat` at `3188ce1`.
Start tag: `ubuntu24-compat-start-20260915`.
Integration branch: `feature/px4sim-baseline-implementation`.
The sim has previously been brought up with `./px4sim restart` and checked with `./px4sim check`; the ledgers in `docs/verification-tests/` are the feature inventory.

## Constraints
All changed repositories must use `feature/ubuntu24-compat` or a branch explicitly derived from it. Preserve user changes. Keep Chimera deploy scoped out unless it blocks the simulator. Optimize rebuilds for cache reuse during development.

## Risks
The ledgers intentionally include orphaned, missing, unknown, and planned items; implementation must not silently turn speculative entries into contract. Runtime tests may be limited by no physical drone and simulator dependencies.

## Approval Required

## Work Packets
1. `px4sim-ui`: implement and test the px4sim front-door/UI behavior in its owning files.
2. `runtime-features`: implement bounded, source-backed 5G Drone and MAVInsight behavior with disjoint ownership.
3. `verification-harness`: add checkpoint/status reporting and focused code/bench/GPS test entry points without duplicating product logic.
4. `review-and-integration`: inspect each packet, resolve conflicts against source, run component tests, commit, and push.

## Integration Policy
Agents edit only their packet ownership. The coordinator reviews diffs, rejects unsupported assumptions, records decisions in packet results, and advances stages only after minimum component evidence exists.

## Verification
Run `./px4sim check`, focused tests for touched modules, restart smoke tests, then the sequestered stage files under `docs/verification-tests/`. Record date, commit, and git status for every tested checkpoint.

## Reusable Artifacts
This workflow directory, its packet notes, and the staged test report form the resume point for future critical checkpoints.
