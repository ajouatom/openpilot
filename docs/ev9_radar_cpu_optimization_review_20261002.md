# EV9 radar/card CPU optimization review, 2026-10-02

## Scope

Review requested after the `000002df--25c01be8dd--0` EV9 investigation.
See `ev9_2df_radar_timing_20261002.md` for the on-device evidence. This
review profiles recorded inputs locally and proposes optimization order;
it does not modify production algorithms, CAN behavior, priorities, settings,
or deployment. The profiled radar_motion and Hyundai CAN/interface code
matches incident commit abe1a232 in the current checkout.

## Device budget: separate computation from waiting

radard's approximately 61.4 ms elapsed work contains approximately 22.55 ms
of CPU execution. The rest is predominantly measured runnable wait on
core5. It is incorrect to treat all 61 ms as radar arithmetic. At 20 Hz,
22.55 ms requires about 45.1% of a core. card already consumes approximately
62.3%, so the pair cannot sustain the intended rates on this core.

This is a software implementation/budget issue, not an inherent requirement
that several dozen radar objects must consume that much CPU. The pipeline
does more than decode range/speed: cross-sensor association, path projection,
stationary-object corroboration, temporal cut-in/out estimation, lead selection,
continuity/latches and output construction.

## Radar profiling

The full cereal log provides 990 model frames. Settings in initData are
EnableRadarTracks=3, EnableCornerRadar=2 and RadarTrackFlip=0. Replay uses
DPathRadarController with production_live_tracks=True, the recorded model
and raw liveTracks, model exposure timestamps, latest preceding carState
and livePose, and measured publication skew. Front measurement delay is zero.

This is a model-paced workload replay, not exact reconstruction of the
overloaded consumer's selected inputs: live radard consumed only 813 frames,
and its precise liveTracks/pose subscriber receive times are unavailable.
In particular, median history length and per-frame work can differ at 20 Hz
versus the recorded approximately 16 Hz. No vehicle behavior equivalence is
claimed. Parsing and input preparation are outside timed sections; IPC,
radarState serialization, Params storage and device scheduling are excluded.

Three unprofiled PC runs, excluding the first 30 iterations, measured means
3.665, 3.676 and 3.765 ms/frame. These are Windows CPU measurements, not
predictions of C4 latency. cProfile adds substantial overhead and its time
shares are hotspot indicators, not measured ARM cycle percentages.

| Function/subsystem | Profile cumulative time | Share of controller update |
| --- | ---: | ---: |
| Controller update, 990 calls | 9.464 s | 100% |
| TrajectoryCutInDetector.update | 5.470 s | 57.8% |
| _median_slope, within cut-in | 2.011 s | 21.2% |
| Primary matching entry | 1.132 s | 12.0% |
| Compact primary cut-out predictor | 0.712 s | 7.5% |
| _vision_supports, within cut-in | 0.694 s | 7.3% |

Rows overlap and must not be added. Path projection is shared across these
subsystems: 100,964 public projection calls and 36,354 cache-miss executions
occur. Existing geometry and projection caches are already present; adding
a generic cache is not a new solution by itself.

There are 68,125 median-slope calls, approximately 69 per model frame.
_median_slope selects a history window, extracts times and values, builds
all pair slopes separated by at least 0.10 s and takes their median.
The main windows are 0.90 and 0.35 s, with up to 1.50 s of retained history.
Its pair loop is quadratic in samples per history. _motion_metrics also
selects a window before passing it to _median_slope, which selects again.
Lateral and longitudinal slopes repeat pair/time work on related histories.

Other repeated work includes re-reading the same model lead fields through
Cap'n Proto for each candidate and recreating lists/tuples for short/long
history statistics. These are better first targets than altering driving
thresholds or replacing the robust estimator with a two-point derivative.

### Radar optimization order

1. Prepare model lead numeric fields and path geometry once per update.
   Preserve candidate-dependent missing/nonfinite fallbacks. Share window
   extraction and time-pair selection across lateral/longitudinal metrics.
   Preserve history insertion, identity transfer, expiry and latch updates.
2. Put measured numerical kernels behind a narrow native boundary, preferably
   the existing Cython build infrastructure: history statistics/median slopes,
   followed by batched path projections if still material. Pass numerical
   arrays/buffers for a whole track or frame, rather than making a native call
   for every scalar operation. Retain the original pair eligibility and exact
   median definition; preserve ordering/ties and float64 behavior.
3. Profile again before translating the rest of the controller. The profile's
   approximately 21% median-slope share means that accelerating only that
   helper cannot justify a claimed 50–80% reduction of the whole pipeline.
   Allocation, matching, branching, conversion and publication remain.

Replacing median slopes with least squares, shortening history, processing
only the current lead, dropping unmatched targets, or lowering the update rate
changes detection behavior. Such work is an algorithm revision requiring its
own evidence, not an equivalent performance optimization. In particular,
future lead handoffs need the histories of presently unselected objects.
The existing cut-out predictor is already in compact production mode; removing
its state maintenance cannot be assumed harmless.

## card profiling

The repository CANParser and CANPacker are Python implementations. get_raw_value
and set_value iterate over signal bytes/bits; MessageState.parse iterates over
signals; parser.update maintains vl, vl_all and timestamps; packer.pack performs
signal lookup, rounding, counter selection and checksums. This is not merely
waiting for CAN hardware, nor an already fully native decoder.

A local interface/controller workload replay joins all 5,983 recorded
carState.radarInput CAN batch boundaries. It uses recorded CarParams and
initial settings via an in-memory Params substitute, the fingerprinted log
message, and latest preceding carControl/modelV2/radarState. ControlsReady is
set for offline registration and apply begins after startup initialization.
This is not full card/process replay: live IPC, Params I/O, exact control receive
schedule and some startup registration timing are not reproduced. A replay-only
BLINDSPOTS_REAR_CORNERS missing-message warning occurs; live CAN is valid.
Therefore the profile is representative code-cost evidence, not a validation
of the original vehicle outputs or exact device cost distribution.

Over 4,782 iterations after 12 s, unprofiled mean interface update plus
controller apply is 0.629 ms on this PC (0.303 update / 0.326 apply).
The on-device stage measurements remain the reference for vehicle cost:
approximately 2.15 ms interface update and 2.24 ms apply near the incident,
within approximately 6.18 ms total card stages.

| Local cProfile component | Cumulative time | Share of approximately 6.05 s |
| --- | ---: | ---: |
| CANParser.update | 1.852 s | 30.6% |
| CANPacker.pack | 1.198 s | 19.8% |
| CarState.update_canfd | 0.844 s | 14.0% |
| numpy.clip calls across the stack | 0.386 s | 6.4% |
| Hyundai CAN-FD checksum, within parser/packer | 0.213 s | 3.5% |

Some rows overlap. Counts include 789,720 get_raw_value calls, 557,872
set_value calls, 73,172 checksum calls and 64,649 clip calls. Scalar NumPy
dispatch is measurable here; changing it requires retaining NaN, bound and
rounding behavior, not simply replacing every clip with min/max.

### Small native-CRC feasibility experiment

A scratch-only replacement uses Python's built-in C implementation
binascii.crc_hqx for the same XMODEM CRC over payload bytes from index 2,
then the low/high address bytes, retaining the exact length-dependent XOR.
Only checksum callbacks are replaced; no production source is changed.

Across two otherwise identical reconstructed runs, all 5,983 CarState
dictionaries and generated CAN message lists match between implementations.
This is variant-to-variant equality, not equality with recorded live TX.
One unprofiled pair measured 0.629 -> 0.606 ms, approximately 3.7% less total
interface/apply time. It is a single desktop comparison, not an established
vehicle speedup or proof across platforms/malformed inputs. It demonstrates
that a native helper is feasible, but CRC alone cannot solve core5 overload.

### card optimization order

1. Precompute immutable signal layout/masks and counter/checksum references.
   Avoid rediscovering metadata on every pack. Reduce repeated conversion and
   dictionary/list work while preserving observable parser API semantics.
   Consider clearing only lists touched on the previous tick, with empty-batch
   and retained-reference equivalence tests; blindly stopping vl_all clears
   changes button-event behavior.
2. Batch native decode/pack by message or CAN input batch. This also benefits
   radarcan, which uses the same parser. Keep signed values, endianness, FD
   lengths, rounding, counter/history/freshness checks, optional registration
   and every output byte equivalent. Converting one bit helper alone leaves
   Python signal loops and dictionary updates behind.
3. Replace selected scalar NumPy operations and repeated lead-display
   extraction only where their semantics and measured cost justify it.
   Do not rewrite all of card: vehicle policy and command generation can stay
   readable Python around narrow, tested numerical/DBC kernels.

## Recommendation and acceptance

Prioritize equivalent cut-in history/statistics optimization in radard,
then native CAN decode/pack for card/radarcan. A small CRC improvement is a
useful contained first native experiment, not the principal remedy.
Keep core redistribution as a separately measured mitigation, without raising
radard above card or weakening frequency/validity checks.

Aim for headroom rather than merely crossing 16 Hz: provisional engineering
targets are card about 4–5 ms at 100 Hz and radar CPU about 12–15 ms at 20 Hz.
Their combined arithmetic demand would be about 64–80% of a core before
other work. These are budget targets, not demonstrated achievable results.
For comparison, a 20% radar-only reduction still leaves card+radar demand
around 98% of one core at 20 Hz, with essentially no margin.

Before deployment, compare every lead/list/validity/state transition and CAN
byte against the reference, including recorded modes 0/1/2/3, empty inputs,
counter/checksum faults, missing/stale frames, source handoffs, stationary
targets, curves and cut-ins. Then measure device CPU and elapsed p95/p99/max,
20 Hz output, receive age, camera/control timing and C3/C4 behavior under load.
The desktop profiles do not establish on-device improvement. Any production
radar/shared-parser changes must also follow the NAS replay deployment and
actual-result verification requirements in AGENTS.md.

Private reproduction scripts/profiles are archived under
`.analysis/archive/2026-10-02/ev9-radar-cost/`. Raw rlog and settings are not
included in tracked documents or committed artifacts.
