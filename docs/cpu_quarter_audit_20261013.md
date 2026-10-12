# Three-month runtime CPU audit, 2026-10-13

## Result and scope

There are further optimization opportunities outside the signal observer. The
best-supported first targets are constant MPC cost preparation, repeated
longitudinal forecast arithmetic, and CAN parser bookkeeping. A blanket removal
of dictionary lookups or input checks is not supported by the measurements.

This task is an **audit with isolated experiments**, not a production code change.
Existing thresholds, control decisions, radar histories, configured rates, CPU
placement and vehicle installation remain unchanged. The preceding production
optimization is documented in [the first CPU review](cpu_hot_path_review_20261013.md).

The audit covers commits with committer dates from July 13 through October 13,
2026, reachable from `carrot-nav` at `f2d8f11332` and `carrot-wip` at
`86d1e21501`. It includes merged ancestors and first-parent merge diffs, rather
than only recent first-parent commit titles. The local repository was shallow;
fetching history since July 12 moved the reachable boundary before the audit
window. The inventory contains:

- 1,132 unique commits: July 208, August 236, September 511, October 177.
- 3,490 touched paths, including historical moves, deleted files and non-runtime
  material.
- 732 current source candidates after excluding test/tool/document directories;
  this includes native headers, vendor and inactive code. Python candidates were
  screened for repeated calls and conversions.

This is an inventory/static screen of that set, **not individual profiling of
732 files or a claim that every change was written by Codex**. Manual tracing and
measurements focused on radar fusion, CAN parsing/vehicle interface, longitudinal
and lateral planners, controlsd, Carrot state/parameter handling and their native
boundaries. Commit identity alone cannot establish which author used an AI tool.

## History relevant to the current runtime

| Period / commits | What was checked |
|---|---|
| July: `f324be4e95`, `70ea8c8893`, `99319aa7b0` | Physical dPath radar replaced the learned radar path; path geometry/projection caches were added; steering-ratio validation reads live parameters |
| August: `13a5191194`, `b7db9cc84b`, `d010f36f31` | Existing front-motion prediction sharing; trajectory cut-in fusion and continuity; distinguish old/superseded occupancy experiments from the running detector |
| September: `67765ebe62`, `a2307bd0df`, `66d975af76`, `eb58e6d7b0` | Lead response in MPC costs, projected gap history, radar/CAN process isolation, live manual steering ratio |
| October: `bc0aa059cc`, `be1a9442f0`, `910bd82fb7` | Native CAN/radar kernels already present; distance-band gap prediction; braking-distance memory and front-radar matching |
| October 13: `f2d8f11332` / `86d1e21501` | Prior signal/impact/diagnostic/coasting optimization retained as the new baseline |

The underlying MPC weight-writing loops predate this three-month window. Recent
changes in the same files make them relevant audit targets, but they are not a
newly introduced Codex regression. Radar statistics and CAN packing already use
Cython; the October 2 pre-native Python profiles must not be presented as current.

## Prioritized findings

### 1. MPC weights: fixed work repeated every update

In `controls/lib/longitudinal_mpc_lib/long_mpc.py:set_cost_weights`, each call
performs 12 interpolations of the same timestamp grid and sends 13 W matrices
plus 12 Zl arrays to acados. The grid interpolation is independent of traffic,
speed, lead response and model output. Precomputing those 12 factors preserves
the current arithmetic, including the terminal matrix's existing construction.

The lateral planner calls `lat_mpc.set_weights` each model update. That method
creates a matrix and sends it 33 times, although its five tuning values usually
remain unchanged between parameter updates. Longitudinal costs can also remain
unchanged, but response ramps and mode transitions must be respected.

Two distinct improvements should be separated:

1. Precompute the fixed longitudinal interpolation factors. An isolated trial
   produced byte-identical solver matrices for 1,000 varying cost vectors.
2. Only resend a complete cost set when its effective values change. This is
   still a proposal: invalidate on solver construction/reset/replacement, changes
   in mode or any effective cost, and preserve the first write. NaN must not
   accidentally be treated as a reusable value. Continue updating state, lead
   response and solving at the existing cadence.

This does not authorize reducing MPC iterations, changing the horizon or skipping
solves. A native solution/output regression is required before production use.

### 2. Longitudinal gap forecasts: repeated arrays and empty prediction work

`long_mpc.py:update` around the gap forecast constructs several duplicate arrays:

- `np.diff(T_IDXS, prepend=0.0)` repeats the module's fixed time differences.
- The same `desired_follow_distance(...)` arguments are evaluated twice per lead
  for the ordinary and braking memories.
- The ego safe-distance array is recalculated in both lead iterations.
- Physical gap arrays are recreated for separate consumers in the same update.
- `getattr(carrot, 'lane_change_gap', LaneChangeGapPlan())` constructs the default
  object even when the member exists. This is small but unnecessary work.

`longitudinal_braking_gap.py:BrakingGapState.margins` copies the state and traverses
all 13 horizon points even when `floor is None`; `_advance` then immediately
returns on each point. A direct fill with **the current margin**, not an assumed
zero, is an audit candidate. A just-released state can retain a small nonzero
margin after its floor clears. The isolated comparison included 0, 0.005 and
1.0 margins and matched bytes for valid-shaped inputs.

Production validation must cover resets, active-to-inactive transitions, both
leads, retained margin, mode changes, finite/invalid inputs and stronger braking.
The existing live state updates and reset calls cannot simply be skipped. The
second solve that withdraws unsafe comfort relief must also remain intact.

### 3. CAN parser: housekeeping remains in Python after native decoding

The current `opendbc/can/parser.py:update` clears every subscribed `vl_all` list
on each batch, even when most messages did not arrive. In one replay's measured
4,793 updates, the profiler counted **2,032,232 list clear calls**, about 424 per
interface update. Three parsers had 29 subscribed messages / 438 signals in total.

The native decoder and packer do not remove Python signal scaling, validity,
counter handling, dictionary publication and list management. In this profile:

| Component | Profile cumulative time | Approximate share of update + apply |
|---|---:|---:|
| CANParser.update | 1.356 s | 28.3% |
| NumPy clip dispatch across callers | 0.386 s | 8.1% |
| CANPacker.pack, including native calls | 0.326 s | 6.8% |
| `getattr` internal time | 0.055 s | 1.1% |
| dictionary `get` internal time | 0.053 s | 1.1% |

Rows overlap and must not be summed. Profiling overhead changes these shares;
they are prioritization evidence, not ARM utilization percentages.

An optimization candidate is tracking only lists populated in the preceding
batch, while retaining list identity and `vl_all`/`all_vals` aliasing. It must
handle empty batches, duplicate messages, failed checksums/counters and dynamically
added subscriptions. Do not cache `can_valid` as a pure getter: it currently
updates `can_invalid_cnt` and can emit rate-limited warnings. Neither CAN checks
nor the per-frame transmit counters may be removed.

### 4. Scalar numerical dispatch in the 100 Hz vehicle path

The same profile counted 67,455 `np.clip` and 17,128 `np.interp` calls in 4,793
updates. Some operate on scalar steering/acceleration values with fixed tables.
Specialized scalar calculations or prepared constant tables are candidates for
more benefit than attribute cleanup alone.

Replacing every `np.clip` with `min/max` is not equivalent for all values. Preserve
NaN propagation, reversed bounds, signed zero, type conversion, rounding and
boundary interpolation; verify final actuator values and CAN payloads. This
audit does not establish a speedup for such a replacement or recommend changing
the steering equations.

### 5. controlsd: filesystem Params reads inside the control loop

`controlsd.py:state_control` reads steering ratio, smoothing, actuator delay and
enable/reboot settings each cycle; lane-line settings add a conditional read.
`CarrotControls.lat_suspend_control` also reads its angle setting every call.
`Params.get_*` ultimately reaches `Params::get` and `util::read_file`; this is
different from an in-memory Python dictionary lookup.

Six representative reads cost about 270 us together on the offroad little-core
measurement below. This is not a measurement of core6 or complete controlsd.
The safe first candidate is moving reads after conditions that prove their
value is unused in that cycle, while keeping immediate reads when needed.
For example, the suspension entry threshold is irrelevant when the entry test
cannot run; its timers and release policy must still execute.

Changing refresh from 100 Hz to 1 Hz would change live-setting response timing
and is outside the requested behavior-preserving scope. A generation-notified
or shared-memory parameter facility is a separate design task with freshness,
failure and atomic-update requirements. In particular, do not slow or cache away
the final `ImpactDashcamReboot` control boundary.

### 6. Radar: optimize the native boundary, not another generic cache

Current native-backed radar profiling still shows substantial work in primary
matching, trajectory cut-in, Cap'n Proto snapshot construction and path projection.
In the busiest tested segment, their cumulative profile times were respectively
0.770, 0.632, 0.322 and 0.350 s, inside 2.984 s of controller updates. These overlap.

Candidates for a further dedicated radar experiment are shared numeric model
lead input preparation and batching history extraction across long/short/lateral/
longitudinal statistics before entering Cython. Preserve pair eligibility,
median definition, arithmetic precision, missing-value fallbacks, track identity
and history maintenance for objects not currently selected as a lead.

Two simple alternatives were actually tested and **not promoted**:

- Copying model lead arrays once per frame for `_vision_supports` did not produce
  a consistent reduction in whole-controller time; copying full arrays adds work.
- Adding a per-frame projection dictionary on top of the existing LRU cache also
  failed to improve total time consistently, despite cache hits.

Both experiments matched all output fields on 2,414 recorded-input updates each.
That equality does not turn an unhelpful cache into an optimization.

The two `_scoped_motion_points` calls observed with corner sensing have different
target sets: corner motion versus front cut-out tracking. In front-only operation
the result is already shared. Removing one wholesale is not an equivalent fix.
The multiple snapshot calls likewise correspond to distinct sensor-delay batches.

### 7. Smaller cleanup remains, with first-call semantics to preserve

Carrot's following-time helpers still use `getattr` around owned `_tf_*` state;
some fields are constructor-initialized and can be accessed directly. Others
derive their initial value from the first live target. Initializing those to an
arbitrary constant would change the first braking/following transition. Use an
explicit unset state where necessary and verify cold-start behavior.

This is a lower priority than the measured loops above. Keep optional schema,
offline adapter, malformed input and actual library capability checks where they
represent a real boundary.

## Isolated measurements and limits

The vehicle was offroad. An independent SCHED_OTHER/nice19 process on core0 used
the real installed acados Cython bindings, current extracted cost methods and
in-memory trial functions. It published no control messages and changed no
production source, setting or service. Three repeated thread-CPU measurements:

| Workload | Current mean | Isolated trial mean |
|---|---:|---:|
| Long MPC weight construction + 25 native cost writes | 455.74 us | 201.45 us with fixed interpolation factors |
| Lateral MPC weight construction + 33 native cost writes | 182.36 us | No changed-cost cache benchmark; proposal only |
| One empty braking-margin forecast | 102.82 us | 8.61 us with current-margin fill |
| Six control Params reads | 270.42 us | No replacement implemented |

These are component costs on a little core, **not total planner speedups, core4/6
utilization reductions or closed-loop validation**. The cost-writing benchmark
does not solve MPC. Do not add percentages from different functions or extrapolate
PC timing directly to vehicle cores.

Radar replay used 578, 637 and 1,199 model frames from three previously collected
morning segments, with mean raw object counts 10.8, 16.5 and 24.2. Current controller
means were roughly 0.35, 0.66 and 1.04 ms on this Windows PC. This uses current
Cython kernels, model-paced latest preceding inputs and mode 1; it excludes IPC,
publication, scheduler waiting and exact live subscriber delivery order. It is
not a general result for every vehicle, object density or radar mode.

The CAN/interface replay joined 5,992 complete recorded batch boundaries; one
boundary outside the available segment was omitted. After warmup, 4,793 update +
apply iterations averaged 0.564 ms on PC (0.274 update / 0.290 apply). Recorded
initial settings were substituted in memory; timing excludes production Params
I/O, IPC and process scheduling. Startup replay state and later live setting
changes are not fully reconstructed. No new CAN-output equivalence claim is made.

## Next implementation order

1. Precompute fixed MPC factors and reuse same-cycle distance arrays.
2. Add an exact inactive braking-forecast path and changed-cost transmission,
   with reset/mode/output regression against the current native solver.
3. Prototype touched-list CAN clearing and scalar dispatch reductions; require
   complete message/counter/validity/actuator comparison before promotion.
4. Profile the remaining radar snapshot/statistics boundary under heavier object
   workloads. The two rejected cache trials are not production candidates.
5. Consider parameter delivery separately; preserve the existing live update
   timing and urgent inhibit behavior.

The report is published to nav and wip with single-parent commits. There are no
runtime modifications or vehicle deployment in this audit. Detailed inventory,
private replay inputs references, profiles and scripts are indexed under
`.analysis/archive/2026-10-13-quarter-cpu/INDEX.md`. Recordings remain private.
