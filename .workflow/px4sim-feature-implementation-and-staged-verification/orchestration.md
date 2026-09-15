# Orchestration: px4sim feature implementation and staged verification

## Execution Rules

- Keep the original objective intact.
- Ask for approval before risky, expensive, external, or destructive actions.
- Keep immediate blocking work local.
- Delegate only bounded, disjoint, materially useful packets.
- Integrate packet results before final verification.

## Branching Rules
Tag the starting commit before changes. Work on `feature/px4sim-baseline-implementation`; any nested repo changes remain on branches derived from `feature/ubuntu24-compat`. Push every accepted increment. Do not force-push or rewrite history.

## Packet Prompts
### px4sim-ui
Inspect the px4sim script, compose/layout/config front door, and ledger. Implement only concrete UI/front-door features that can be verified locally. Preserve cache-friendly rebuild behavior. Report files, tests, and unresolved ledger items.

### runtime-features
Inspect 5G Drone and MAVInsight source/docs and implement one coherent, source-backed runtime slice with disjoint ownership. Prefer simulator-verifiable behavior. Report files, tests, and assumptions.

### verification-harness
Inspect existing verification docs/scripts. Add or improve stage-specific, sequestered test entry points and checkpoint reporting. Do not claim runtime success without evidence.

### review-and-integration
Review packet diffs for regressions, branch compliance, and ledger alignment; run focused tests and record accepted/rejected decisions.

## Completion Audit
Before each commit: `git diff --check`, focused tests, `./px4sim check` when relevant, status clean except intended files, commit message describing the slice, push branch. Before stage advancement: record evidence in the report and preserve failed checks as risks.
