# USB wrap backport and recorded curve-policy comparison, 2026-11-04

## USB correction

Reviewed upstream [openpilot ad4c9cdf / PR39204](https://github.com/commaai/openpilot/commit/ad4c9cdf1857034bb06c12e4b0cfba093e6ae3d6)
and the actual tinygrad fix [42ea17d1 / PR19138](https://github.com/tinygrad/tinygrad/commit/42ea17d19735bc746175fa03a0c43162863d0b36).
The latter changes the USB zero-fill matcher from a bare zero to zero-or-cast-zero.
SDMA ring wrap can require zero padding; a cast zero previously missed USB lowering
and could reach host memset with an address belonging to the USB GPU. Upstream
also replaces an ineffective ring-wrap test with one that positions the queue
at its final dword and forces an actual wrap.

The openpilot commit upgrades d39227 to ec91b74 and recompiles the Chestnut PKL
and both warp artifacts. Those runtime revisions span 47 tinygrad commits, so
upgrading the runtime under our existing PKLs would not be an exact small fix.
Instead backport the one matcher correction in memory, before artifact loading,
only for verified runtime cb9bf2cf (currently Light speed/d39227). The fixed
rule precedes the old rule, is idempotent, and preserves upstream USB fill
semantics. No extracted runtime file or immutable package hash is modified.

Source/hash inspection of the three NAS runtime archives confirms N16/90970e01
and Flip/c7c58622 use an older ranged-store implementation, including its own
zero-value handling; they do not contain this defective memset matcher and are
not patched. Model weights and their runtime pairings remain unchanged. The
Light speed artifact's pickle opcode metadata was inspected without execution;
it contains no memset string. The affected copies are lowered at runtime.
Light speed's AMD warp uses a new usb-wrap-v1 cache filename so an earlier cached
host dispatch is not reused. Old files remain untouched. The device validation
receipt now includes the USB hook sources, requiring a fresh startup check when
the hook changes. GPU ownership, timeouts, output validation and recovery remain.

The real pinned d39227 PatternMatcher reproduces the missing cast-zero match;
the backport routes both bare and cast zero through USB fill while leaving a
nonzero fill unmatched. This runs without an AMD device; it is a compiler-rule
regression check, not a physical ring-wrap or sustained-inference test. No claim
is made that this caused or cures the earlier EV6 timeout/Flip TLB failure.

## Inputs and comparison limits

26 full rlogs total 346,631,805 compressed bytes. Sixteen were copied from their
NAS locations with source/copy hashes matched; ten recent morning logs were
reused after rechecking the exact hashes from their earlier source-verified
manifest. Full cereal decoding finds 30,284 model messages; 30,276 have usable
finite geometry and fresh valid carState (eight excluded). Of those, 25,243 have
vehicle speed at least 10 km/h and form the table denominator below.

All 30,284 messages have **no orientationRate.zStd and no rawPredictions**.
There is no way to recover actual yaw-rate uncertainty from these recordings.
Position std is not substituted for yaw-rate std, and no guessed value is
presented as a model output.

The corpus includes Light speed and Flip Flop eGPU windows, the EV6 internal
fallback period, a recent JoiningModel/Jetlink route, and 4,798 usable frames
from historical retired MD v3 recordings. It is not a clean three-current-model
benchmark. JoiningModel alone does not attest the exact active external weights;
no unrecorded N16 identity is asserted. Per-segment source, backend, model hashes
where available, settings and exclusions are retained in private outputs.

`tools/modeld/compare_curve_policies.py` compares:

- Current Carrot distance-based speed ceiling, using each log's stored curve
  sensitivity/lower-limit settings and recorded speed ratio/acceleration.
- Exact pinned ssh-model f3bf59fd acceleration-cap reference, without std.
- Each of those methods with constant hypothetical yaw-rate std of 0.01, 0.03
  or 0.05 rad/s, added to absolute yaw rate (preserving direction for Carrot).

The algorithms differ beyond uncertainty: Carrot uses a 1.9 m/s2 lateral budget
scaled by sensitivity, a configured minimum curve speed (18/35/44 km/h in these
recordings), path arc length, a reaction/jerk distance reserve and median geometry.
The reference uses 2.5 m/s2, forward plan x, no corresponding configurable speed
floor, neighbouring-point support and distinct -2/-0.5 m/s2 caps. Therefore a
change in onset cannot be attributed solely to uncertainty or one threshold.

These are **open-loop evaluations of the same recorded input**, not a replay
of changed vehicle motion or measured braking. Other planner candidates, driver
input, actuators and road outcomes are not simulated. Each supplied segment
cold-starts policy state; gaps above 200 ms reset it. Settings are snapshot values,
not proof that no setting changed mid-segment. Histories must be warmed when
interpreting individual onsets. CSV rows and full summaries remain private.

## Results

The two columns measure different interfaces. The existing ceiling counts frames
where it is over 1 km/h below the current cruise-unit vehicle speed. The reference
counts acceleration caps below -0.1 m/s2. Neither is a count of actual brake events.
Do not interpret their difference as a measured safety improvement.

| Hypothetical yaw std (rad/s) | Existing ceiling below vehicle speed | ssh-model negative acceleration cap |
| --- | ---: | ---: |
| 0 (recorded mean geometry only) | 1,746 / 6.92% | 1,769 / 7.01% |
| 0.01 | 2,244 / 8.89% | 1,962 / 7.77% |
| 0.03 | 2,851 / 11.29% | 2,997 / 11.87% |
| 0.05 | 3,643 / 14.43% | 3,921 / 15.53% |

The reference is not uniformly earlier or stronger. In the latest supplied
Flip Flop segment, existing lower-ceiling episodes begin at +34.80 s and +40.65 s;
reference negative-cap episodes begin at +34.05 s and +42.55 s respectively.
Thus one starts earlier and another later. These are policy thresholds, not
physical braking onset times, and speed-ceiling/cap thresholds differ.

With assumed std 0.03, the same reference episodes start at +33.70 s and +40.25 s
(0.35 s and 2.30 s earlier than its own zero-std case). The existing method's
+34.80 s episode becomes a longer +33.90..45.06 s interval; its +46.60 s episode
starts at +46.05 s. Uncertainty can advance and connect restrictions rather than
merely lower the speed at the peak of a curve.

It can also add restrictions on weak-curvature predictions. On frames at least
40 km/h where all usable 0.5-6 s mean-plan curvatures are below 0.001/m, assumed
std 0.05 adds 324 existing-ceiling bindings and nine reference negative caps that
were absent at std zero. At std 0.03 the corresponding counts are 31 and zero.
This identifies sensitivity on the model's near-straight predictions, not proven
false positives on visually confirmed straight roads.

The reference has another non-monotonic detail: it allows up to -2 m/s2 for a
future curve more than 1.2 times the current curvature, but only -0.5 otherwise.
Adding std also changes that classification. At assumed std 0.03, 36 moving
frames have a cap more than 0.1 m/s2 *less* restrictive than the zero-std case
(maximum difference 0.8 m/s2). Do not assume more uncertainty always yields more
braking when importing the complete policy.

## Disposition

Keep live curve control unchanged. No parameter, curve gain, acceleration cap,
following gap or steering policy is activated by this work. The offline reference
is not imported by any driving process. Actual measured uncertainty is needed
before deciding whether to integrate uncertainty into the existing envelope or
replace it with the reference acceleration-cap policy.

Publish the already-computed plan orientation-rate x/y/z stds in the existing
modelV2 fields for the next recordings. No schema change or new model output is
required. Full-schema tests for N16/Flip/Light contracts verify correct axes,
serialization, unchanged mean predictions/action and all other message fields.
This does not make uncertainty influence a controller. Normal comma Git Pull
and restart delivers both the USB backport and this additional logging; no new
model download or Jetson host bundle is needed.

Desktop checks: 128 focused model publication/runtime/curve/recovery tests pass,
plus the actual pinned d39227 matcher regression. Lint and diff formatting pass.
Native Linux compatibility and the broader integration suites are required on
the published source. No vehicle writes, USB/GPU hardware test, CAN operation,
restart or road acceptance were performed. Raw routes and detailed traces stay
under ignored `.analysis/scratch/2026-11-04-usb-curve/`.


Published source: cfd8d7b8668d6dd75098acf441a214eda362d9e4.
[eGPU CI41834213643](https://github.com/ajouatom/openpilot/actions/runs/41834213643)
passes 505 contract/runtime checks, 57 navigation checks, three native old-runtime
checks/two new-runtime-only skips, all five native Light speed checks including
the cast-zero regression, and Web tests/build. Counts overlap local coverage;
these tests do not exercise a physical USB AMD ring wrap.

[Jetson integration CI41834213685](https://github.com/ajouatom/openpilot/actions/runs/41834213685)
passes the five publication/comparison tests, 2330 broad checks/56 skips,
supporting suites 232/62/94/119/11/11, and Windows. User-doc CI41834213931 passes.
These establish compatibility, not Jetson/eGPU physical timing or road behavior.
