# Packet: px4sim UI

Owner: Rawls
Status: accepted for integration

Implemented prepared-image restart messaging and `restart --build` cache-aware rebuild behavior in `px4sim`, plus stale-feed clearing and updated action text in `scripts/tui.py`.

Evidence: `python3 -m py_compile scripts/tui.py scripts/state.py`, `./px4sim check`, `./px4sim help`, and `git diff --check` passed.

Rejected/deferred: dynamic fleet editing, scenario-constrained selection, richer Foxglove-faithful diagnostics, and pager/text wrapping require broader design or runtime evidence.
