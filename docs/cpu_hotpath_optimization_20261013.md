# Equivalent CPU hot-path optimizations, 2026-10-13

This implements the measured opportunities in `cpu_quarter_audit_20261013.md`.
The scope is less repeated CPU work with the same control behavior. Settings
cadence when used, input validation, perception cadence, lead selection, braking,
steering limits, DM protection and model selection are unchanged.

## Changes

- Long MPC computes its fixed acceleration-change interpolation factors once.
  Long/lateral MPC send cost matrices only when their effective double values
  change. Keys distinguish signed zero; nonfinite costs always take the original
  write path. Reset, solver replacement and long mode changes invalidate the
  cache. A failed partial write cannot leave the prior cache valid. Horizon,
  references, constraints and solver calls remain unchanged.
- Long MPC reuses same-cycle desired/physical distance arrays and fixed time
  differences. Empty braking-memory forecasts fill the current margin directly,
  including the small residual that can remain after the floor expires. Active
  forecasts and every state update/reset still execute.
- CANParser clears only signal lists populated by its previous update. It keeps
  list identity, address/name aliases, multiple messages in one batch, dynamic
  subscriptions and counter/checksum/timeout handling. Its internal list is
  independent of the caller-owned returned address set.
- Hyundai call sites already producing `float(np.clip(...))` use a scalar float
  clip with the same NaN, reversed-bound and signed-zero behavior. Array clips,
  integer clips and interpolation remain unchanged. This is deliberately limited
  to float control/HUD fields; no general min/max substitution.
- controlsd reads lane smoothing/delay settings only when an active lane path
  consumes them. Lateral suspension reads its threshold only while testing entry
  with steering pressed. Every needed read remains immediate; steering ratio and
  the urgent reboot inhibit retain their existing cadence.
- Carrot following-time state is explicit. The unset deceleration baseline and
  mode factor still derive from the first live input, preserving cold start.
  Owned initialized fields no longer need repeated getattr defaults.

## Validation

- 1,969 focused desktop tests pass, including compiled CAN/radar kernel checks,
  Hyundai actuation, MPC wiring, braking memory, following time, coasting and
  full control state transitions. Windows substitutes only unavailable hardware/
  Params/IPC dependencies; these are not device-integration tests.
- Broader screening found 16 pre-existing Hyundai fingerprint/database failures
  and one initial CAN-validity expectation failure. The same failures reproduce
  with baseline parser/arithmetic. They are recorded, not silently fixed by
  changing validity or vehicle identification in this performance task.
- Actual vehicle acados Cython solvers, in an isolated offroad core0/nice19
  process: 400 long and 300 lateral updates have byte-identical trajectories,
  solver status and checked planner outputs, including resets and changed costs.
  No production worker, model, control transport or setting was changed.
- Stored Ioniq CAN replay: all 5,992 complete recorded batches match in car state,
  actuator output, outgoing CAN bytes, parser values/all-values/timestamps/raw
  payloads/counters. One batch outside the available segment is omitted. Recorded
  Params are supplied in memory; later live changes are not reconstructed.
- 2,000 complete control updates match output and torque PID/curvature state;
  10,000 follow-time updates match, starting cold and varying speed, mode and
  braking boost. Tests also exercise lane/model paths, MEB, inactive control,
  suspension entry/hold/release and immediate threshold changes.
- Float clip comparisons cover 343 special-value combinations and 10,000 random
  triples. Native timing uses NumPy 2.5.1, desktop tests NumPy 2.5.

## Measured costs and limits

Isolated vehicle thread-CPU means, three repeats, microseconds per operation:

| Operation | Before | After |
|---|---:|---:|
| Long MPC unchanged costs | 452.1 | 6.1 |
| Long MPC changed costs | 452.2 | 210.5 |
| Lateral MPC unchanged costs | 177.4 | 4.9 |
| Empty braking forecast | 102.7 | 8.9 |
| One float clip | 29.1 | 0.9 |

These are component timings on a little core, not whole-process/core utilization
percentages. Native solve equivalence is separate from the timing loops. Recorded
CAN update+apply after warmup takes roughly 0.60-0.64 ms with the changes versus
0.67-0.80 ms before on this PC; runs include scheduling noise and exclude IPC and
production Params I/O. Do not extrapolate these numbers to onroad core6 load.

The additional radar scalar-only model snapshot trial matches 2,414 outputs but
shows only about 0-1% benefit on two segments and about 4% on the busy segment.
Together with the earlier failed full-array/projection-cache trials, this is not
a consistent enough gain to add another cache. No radar matching/history math
is changed. Native batching remains a separate unproven engineering proposal,
not a claimed completed speedup.

Publication targets are carrot-nav and carrot-wip with single-parent commits,
preserving their model/navigation differences and unrelated working-tree edits.
Carrot Routes CI is triggered by the shared opendbc changes; deployment evidence
is recorded privately. Vehicle production installation and onroad CPU/thermal
measurement are not part of this publication.

Private reproducibility scripts and evidence:
`.analysis/archive/2026-10-13-cpu-implementation/INDEX.md`.
