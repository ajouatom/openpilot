# DM road-context reader crash — 2026-09-28

The requested EV6 tmux capture at 18:46:26 shows repeated `dm2d.py:92`
`TypeError: an integer is required` exceptions. SCons and manager startup
completed. The failing expression was `model.orientationRate.z[:10]`; the
following lane-confidence expression also sliced a Cap'n Proto list. The
capture contains 39 occurrences of that exception text, including duplicated
parent/child tracebacks, not 39 independently established crashes. It has no
occurrence of the previous SubMaster constructor assertion. These failures
interrupt driverMonitoringState and coincide with its reported communication
health failures; other shown model/planner services remain alive.

Cap'n Proto DynamicListReader requires integer indexing here. Read the same
first ten yaw-rate values and lane probabilities at indices 1 and 2 using
integer indices, retaining the existing length guards and thresholds. This is
a DM consumer fix; radar detection, lead selection and stock DM policy are
unchanged. No automatic update or reboot is triggered by an onroad DM crash.

Earlier simulated dispatcher tests left road-model arrays empty. Their length
guard short-circuited before the invalid slices, so passing those tests did not
establish compatibility with populated messages. Tests now provide populated
read-only cereal model messages in both camera-present and camera-absent paths,
both modes and touch/BT input cases; empty-model startup coverage is retained.
Before the fix, ten cases reproduce the exact TypeError; afterwards all 113
adapted policy/parser/dispatcher tests pass. Focused Ruff checks also pass.
Desktop adapters replace native IPC/Params/hardware, but cereal readers are real.
This does not establish an on-device restart or post-update driving result.

Private capture location, before/after test output and the reproduction runner
are retained in the ignored local analysis archive, not in public fixtures.
