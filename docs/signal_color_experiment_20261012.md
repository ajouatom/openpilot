# Signal color correction experiment — 2026-10-12

The user requested starting a correction experiment after the morning false-green
investigation. A narrower green-color candidate is implemented and replayed, but
is **offline-only and default OFF**. No vehicle deployment, ONNX retraining,
model x/v change, DM/backend change or control-policy change was made.

## Change

`SignalTracker(conservative_green=True)` limits OpenCV HSV green hue to 38–90
instead of 38–105. Saturation/value, connected-component shape and positional
scoring remain. The same option reaches housing proposals, daytime bright cores,
nighttime halos and causal template tracking. Pure cyan remains eligible;
blue-dominant pixels no longer supply green evidence. No old green is substituted
for an uncertain current observation.

This is a hand-written image-processing experiment, not neural-network training
or a complete signal-housing detector. Narrowing the green mask also changes
proposal competition and can reveal red evidence previously suppressed by mixed
red/blue scores; it does not guarantee identical red detections. Actual road
relevance, arrow interpretation and complete housing alignment remain unresolved.
Road-edge filtering was not added: the preceding probe rejected only 8/62 selected
backgrounds and neither red-signal miscrop.

## Same-input replay

`tools/signal_analysis/replay_color_experiment.py` runs both full causal trackers
on identical decoded HEVC images at the recorded observer capture cadence, with
robust tracking/daytime cores enabled. Exact `roadEncodeIdx` frame IDs join the
images; a delayed observation logged in segment 8 correctly maps to segment 7's
last image. Adjacent available segments preserve tracker history. Missing video
segments reset it. All 3,348 observations map; none are silently approximated.

Sources are the 11 morning 109f segments and nighttime 109d-6/109e-3. The other
collected segments were not replayed because this experiment uses locally retained
full-resolution videos, not the smaller qcamera preview.

| Paired replay | Existing | Candidate |
|---|---:|---:|
| Morning fresh green candidate samples, 2,727 observations | 5,608 | 581 |
| Morning observations containing any fresh green | 1,857 | 342 |
| Morning fresh red candidate samples | 610 | 931 |
| Night fresh green candidate samples, 621 observations | 437 | 419 |
| Night fresh red candidate samples | 203 | 203 |
| Desktop morning mean processing time, ms/image | 9.45 | 7.51 |
| Desktop night mean processing time, ms/image | 10.50 | 10.47 |

The 89.6% morning reduction is **candidate count reduction, not measured false
positive reduction or accuracy**. Repeated objects count repeatedly. Live raw
camera results previously had 6,792 morning green candidates; encoded-image replay
does not reproduce those pixels or detections exactly. Desktop timings use one
OpenCV thread and alternate execution order; they do not establish C4 speed.

## Visual checks and known failures

On the previous deliberately selected 73 candidate boxes:

- 61 background boxes that replayed green become unknown; the remaining background
  was already unknown in the encoded image. This is a development set.
- Six correct red and three correct green boxes retain their colors.
- Both logged red-signal miscrops produce unknown with the candidate. Only
  `f10-0020` reproduces the erroneous green in the old encoded-image scorer;
  `f6-0768` already scores unknown there. Do not claim two independently reproduced
  failures were fixed. The full candidate replay emits no green in segment 6.

After freezing the threshold, 21 additional frames were selected to inspect
retained greens and nighttime removals. Among their old green candidates,
37 visible green-lamp examples remain detected and 29/36 backgrounds are removed.
Seven backgrounds survive: five LED-text samples and two tree/window-reflection
samples. This selection is neither random nor a complete recall annotation.
Green-lamp presence is not proof that the lamp authorizes the ego vehicle to go.

All 18 nighttime removed candidates were additionally inspected in crops and
scene context. One is a **real motion-blurred green signal**, 109e-3 encoded frame
635 while turning. Thus the change has a known sensitivity regression. Other
reviewed sharp near/far nighttime greens remain visible; this does not establish
robustness across all exposure/white-balance conditions.

The unchanged start guard's `observe()` color-confirmation diagnostic was also fed
the replay tracks using the existing three-observation publisher rule and recorded
result age. In morning segment 8 the candidate first confirms green at 21.090 s;
the encoded baseline never confirms it in this probe. The actual live guard had
released at 18.106 s. This diagnostic excludes engagement/validity/vehicle-response
simulation and cannot validate departure timing or show a 3 s causal regression.
It does demonstrate that encoded replay is not an exact live-control reconstruction.

## Checks and disposition

84 focused tests pass, covering existing tracker/guard behavior, blue rejection,
green/cyan and saturated night lamps, causal-template filtering, red-to-green
confirmation, opt-in default and cross-segment frame alignment. The initial test
invocation hit the unavailable Windows native Params extension; tests then ran
with `--noconftest`, without changing production dependencies. Default-OFF output
matches the previously committed tracker exactly on 21 reviewed images.

This is a useful sky-rejection experiment with measured limits, **not a promoted
vehicle fix**. Keep the option off in live workers. Next work should discriminate
actual housings from LED text/reflections, preserve lamp alignment from the red
anchor, and test motion-blurred/cyan greens without re-admitting sky. Keep display
memory separate from fresh control evidence and retain the existing DM GPU guard.

Private evidence: `.analysis/archive/2026-10-12-signal-color-experiment/` contains
paired traces, labels, screenshots, totals and reproduction scripts. Raw drives,
captures and labels remain local. The preceding collection and investigation are
documented in [the morning report](signal_morning_false_green_20261012.md).

## Follow-up: user prioritizes rejecting false green over missed observations

During the experiment the user explicitly accepted discarding turn-time results
and retaining past observations through misses, prioritizing prevention of
red-to-green false classification. Add an offline third variant using recorded
valid carState steering angle/rate and speed, sampled causally before each image.
All recorded yawRate fields in these segments are zero, so yawRate is not treated
as a measured rotation signal.

The experimental `GreenMotionGate` uses these provisional thresholds:

- While moving above 0.3 m/s, absolute steering >=15 degrees or steering rate
  >=90 degrees/s immediately rejects green. Red observations remain eligible.
- Resume after 0.5 s continuously at <=8 degrees and <=30 degrees/s, or after
  0.5 s stopped at <=0.3 m/s. These are experimental gates, not calibrated yaw
  or proof of straight travel. Curved approaches can intentionally lose green.
- Missing/invalid, nonfinite, stale (>200 ms), future or reordered motion context
  cannot grant green. Angle hysteresis prevents repeated reopening near a limit.
- Rejection converts fresh green to unknown before tracker confirmation and
  clears a previously confirmed/pending green immediately. It preserves red
  history. The existing start guard keeps a latched red through unknown/missing
  observations; stored/displayed green never releases it. Existing explicit
  manual gas, disable and gear reset policies remain unchanged.

An initial angle-only version was rejected: the real segment8 stop has steering
-39.5 degrees at zero speed. Calling this a turn would suppress a legitimate
green while stationary. Speed-aware gating retains this observed stop's green.
The blurred nighttime green at 109e-3 frame635 occurs at +160.2 degrees and
3.88 m/s, so it is now explicitly outside the accepted measurement conditions,
consistent with the user's revised priority. This does not prove that all missed
greens occur during turns or remove the need to test straight motion blur.

The same 3,348-image causal replay with the third variant gives:

| Fresh green candidate samples | Existing | Color-only | Color + motion |
|---|---:|---:|---:|
| Morning | 5,608 | 581 | 509 |
| Night | 437 | 419 | 338 |

692 morning and269 night observations are gated (turns, startup, context recovery
or straight/stopped dwell). All961 contain neither a fresh green vote nor a
confirmed green state in the gated tracker. Red counts stay931 morning/203 night,
identical to color-only. Normal green in stopped segment8 remains; the diagnostic
confirmation time remains21.090 s, with the same live-reconstruction limitations
above. Existing variants' candidate counts remain identical to the first replay.

93 focused tests pass, including fresh-context gating, stationary turned wheels,
hysteresis/dwell, future-data exclusion in replay, immediate old-green removal
and an actual start-guard unit sequence that holds red through rejected greens.
LED-text and reflection false greens in accepted conditions remain unresolved.
No blanket claim of zero red-to-green errors or validated autonomous departure is
made. Both experimental options remain offline; the vehicle was not changed.

Private `motion/` contains full-rate extracted carState, `replay_motion/` is the
rejected angle-only replay, and `replay_motion_final/` plus `motion_results.json`
contain the speed-aware results. Reproduce with `--motion-dir MOTION_DIRECTORY`.

## Follow-up: pose as the direct rotation cue

The user then asked to use pose instead of steering angle. The fourth offline
variant uses `livePose.angularVelocityDevice` rotated into the calibrated vehicle
frame through the repository's `PoseCalibrator`, as in locationd/torqued. It uses
calibrated yaw rate, not differentiated absolute heading. Calibration must be
valid and <=1 s old; pose event/measurement, orientation, inputs, posenet and
sensors validity are required. Raw carState yawRate remains unused (all zero).

`GreenPoseGate` rejects green at absolute yaw >=3 degrees/s, resumes after0.5 s
continuously <=1.5 degrees/s, and rejects yaw standard deviation >2 degrees/s.
Both arrival and measurement timestamps are checked: pose must already be
available before image capture, with measurement age <=200 ms. Frozen/reordered,
future, nonfinite or invalid pose resets permission. No steering-based fallback
can turn unavailable pose into a green authorization. These yaw thresholds are
provisional and do not claim a calibrated error bound or no motion below3 deg/s.

Recorded examples make the distinction concrete:

- Night blurred green during a turn: steering+160.2 degrees, calibrated yaw
  about-13.7 degrees/s. Both motion variants reject green.
- Morning signal stop: steering-39.5 degrees, yaw approximately0.0005 degrees/s
  in magnitude. Pose accepts fresh green without a special steering-angle excuse.
- Morning segment6 miscrop: steering7.4 degrees and yaw about-0.61 degrees/s.
  Steering recovery dwell still rejects this frame; pose permits color measurement.
  The color candidate remains unknown, not green. This separates actual rotation
  rejection from the indispensable color/housing false-positive work.
- Segment10 red miscrop: yaw about+4.1 degrees/s, rejected by pose as well as by
  the narrower color evidence.

All3,348 images were replayed again with existing, color, steering and pose
variants. Pose+color produces514 morning/375 nighttime fresh green candidates,
versus steering+color509/338 and color-only581/419. These are candidate counts,
not precision/recall. Pose gates692 morning+206 night observations; all898 have
zero fresh or confirmed green outputs. Red counts remain931/203. The stopped
segment8 real green and its21.090 s diagnostic confirmation are retained.

108 focused tests pass. Existing default output still matches the pre-experiment
tracker on21 reviewed images. The pose variant is the more direct rotation
experiment; steering remains a comparison/diagnostic signal. LED/reflection false
greens remain, so neither variant is promoted or deployed. Private `pose/`,
`replay_pose_final/`, `extract_pose.py` and `pose_results.json` retain the evidence.

## 2026-10-12: opt-in red preparation before model stopping

The user requested a separate `e2eStopPrepare` state: when a convincing red is
observed before the model enters stopping, gradually request about -0.5 m/s²,
announce the existing “신호가바뀌었어요” sound, release when the observation
disappears/changes, ignore leads, and hand over to the original model stop.

Implemented as state6; existing values0..5 are unchanged. A separate startup
opt-in `/data/signal-color-shadow/stop_prepare_enabled` defaults absent/OFF.
This does not enable the old latched moving-red assist. Tracking CPU observation
can publish a dedicated tmpfs endpoint even in comparison mode; display and
stopped-red guard endpoints keep their previous roles. No new model or process
is introduced; additional calibration/camera/device subscriptions exist only
when the planner starts with this opt-in. Turning the flag off releases the
preparation at the existing0.5-second flag poll; enabling initially requires a
planner restart. Do not enable from the present validation results.

Entry requires active/valid longitudinal control in Drive,5<speed<82 km/h,
no gas/brake, no leadOne/leadTwo, no blinkers, and fresh valid calibrated pose
with |yaw|<=1.5 degrees/s and conservative angular standard deviation<=2 degrees/s.
Require one unambiguous same-session/same-ID current red housing for>=0.6 seconds
and>=4 distinct observations, frame age<=200ms and gap<=250ms. Unknown, changed
color, disappearance, stale/invalid context, a lead or driver intervention cancels
the preparation. Gas/brake keeps a ten-second re-entry cooldown. Existing model
stopping/stop hold wins; no forced red trafficState or fabricated model x/v.

Only a currently measured `detected` horizontal housing with quality>=0.5,
red score>=0.4 and green score<0.08 is eligible. Single-core proposals and template
matches cannot establish metric housing size. OS04C10 road-camera geometry,
calibration, modeled road edges and model path reject implausible candidates.
Housing width0.7–2.5m and mounting height3–9m are **assumptions**, not measured
dimensions. The supported distance interval must lie within25–140m, inside both
road edges with0.3m margin and within2m of the modeled path; each road-edge std
must be<=0.5m. Do not extrapolate model curves. These are provisional filters,
not proof of ego-lane relevance, perfect red recognition or stop-line distance.

The final longitudinal output, after coasting, applies an upper acceleration cap
that ramps down at0.25m/s³ to-0.5m/s². It starts at the preceding output (or measured
acceleration); zero to-0.5 takes two seconds, positive acceleration takes longer.
Use elapsed monotonic time so additional radar-triggered updates cannot accelerate
the ramp. Preserve stronger braking, ordinary actuator limits, DM and stopped-red
veto. Keep velocity/jerk outputs and next planner acceleration consistent with the
cap. This is a requested acceleration, not a guarantee of measured deceleration.
No shouldStop/go request is manufactured. Cancellation removes the cap and returns
to ordinary planning; an independently stronger braking request can remain.

The existing trafficSignChanged event provides the requested WAV at entry, with
ten-second re-announcement suppression. Camera overlays keep R/G/? and the stale
`~` qualifier but remove visible track numbers; internal IDs remain for continuity.
The main onroad renderer labels state6 “신호 감속 준비”. Physical sound/display
and onroad integration are not yet device-validated.

Validation:184 focused helper/runtime/planner/display/observer regression tests
pass on desktop. Default-OFF state and targets match the prior source across
1,200 synthetic input frames. The existing3,348-image night/morning candidate
replay was screened against the new gates:1,134 fresh red candidate samples,
only6 strict measured-housing red samples, **zero geometry-qualified samples and
zero preparation entries**. All six have at least one road-edge std>0.5m; several
also have insufficient modeled path range. This is a retained-observation/context
screen, not a new full camera/native-MPC/vehicle replay; it cannot establish false
positive rate or improved stopping. The helper is implemented but current detector
evidence does not justify activation. No vehicle was connected or modified.

Private reproducibility evidence is retained under
`.analysis/archive/2026-10-12-stop-prepare/`; dependencies are disposable. The prior
night-log numerical stopping-distance review is separately retained under
`.analysis/archive/2026-10-12-red-stop-target/` and is not native acados validation.

### Follow-up: use spatial road edges / lane lines, not position range

The user requested constructing and checking the geometry with `roadEdges` or
`laneLines` x/y/z, because the time-indexed `position` shrinks while stopping.
`housing_range` now has no position input or path-range requirement. Use the
two inner lane lines when both probabilities>=0.8 and lateral std<=0.5m; otherwise
use both road edges with std<=0.5m. All curves must have finite, strictly increasing
x and consistent x/y/z lengths. Lane widths2–5m and road widths2–20m bound obvious
geometric failures. Interpolate z at each distance hypothesis and do not extrapolate.
When both sources are trustworthy, intersect their lateral bounds; conflicting
geometry is not silently ignored. Road edges alone are not ego-lane assignment.
The previous 3–9m height,25–140m distance-range, measured-housing/color/temporal
requirements remain experimental and unchanged.

Reading the real rlogs also exposed an input-adapter defect from the initial
preparation implementation: livePose angular velocity is an XYZMeasurement with
x/y/z and xStd/yStd/zStd, not value/std lists. The old reader failed closed on its
AttributeError. It is corrected here; the runtime regression now uses an actual
Cap'n Proto LivePose reader rather than a permissive fabricated structure. This
does not affect the earlier independent pose replay, which used PoseCalibrator.

Paired check reuses the3,348 original-cadence candidate records and reads full-rate
model messages (including lane lines), calibration, car state, radar, engagement
and pose from the13 original rlogs. It is still a recorded-input eligibility
screen, not native MPC or vehicle-response validation. Observer pixels are not
rerun; its input candidates remain the prior pose/color experiment. Driver mode,
user stop-distance and all entry-policy histories are not reconstructed, so the
screen must not be used to authorize activation even if a future entry appears.

Results:1,134 fresh red samples;94 measured-housing red samples;6 pass the strict
color/state requirement. Old geometry can provide a trusted road pair for none
of those94; the new lane fallback provides a corridor for9. All9 have possible
height hypotheses, but all fail the lateral corridor test. Neither old nor new
geometry accepts a complete candidate; preparation entries remain0. The six
strict red samples still have no trustworthy road/lane pair. These counts are
repeated observations, not intersections or an accuracy score.

Exact video-frame review of109f-6index764 and109f-7index103 confirms real red lamps
inside the detected boxes. They are NOT examples of false-red rejection. Their
failure demonstrates that requiring the signal to project directly over the
inferred ego lane can discard real signals; the housing-size distance hypotheses
also remain uncertain. This supports the user's observation of distant detection,
but does not establish that either particular lamp governs the ego lane. The
filter must not be described as an accuracy improvement or enabled for braking
from this result. Geometry can support height estimation without solving signal
applicability; broadening lateral permission needs separate negative validation.

187 focused tests pass, including edge/lane fallback, conflicting sources,
non-extrapolation, camera rotation, road-height changes, and real pose-message
input. Feature remains default OFF; no vehicle connection or deployment.
Private before/after data, exact review PNGs and the reproduction script are in
`.analysis/archive/2026-10-12-signal-road-geometry/`.


### Follow-up: lateral allowance outside the ego lane

The user clarified that the signal search region must extend to both sides of
lane boundaries. The previous exact-lane corridor was too narrow. Expand each
inner lane boundary by **3.5m per side** (roughly one extra lane), in road
coordinates at each distance hypothesis, not a fixed pixel rectangle. This is a
provisional engineering allowance, not an optimum fitted to these few samples.
Road-edge fallback retains its 0.3m inward clearance. When both sources are
reliable, reject disagreement of the ORIGINAL lane/road regions before expansion,
then clip the expanded lane region to the road bounds. Do not expand uncertain
geometry or extrapolate curves. A lamp accepted here is only a candidate; the
region does not establish which lane it controls. Other color, height, distance,
pose, lead, freshness and confirmation requirements remain unchanged.

Repeated the retained-candidate/full-rlog screen on all 3,348 observations from
13 night/morning segments. There are 94 fresh measured-housing red candidates,
86 green and 6 unknown. These are repeated observations, NOT separate lights.

| Extra allowance per side | Red observations with lateral/height support | Red observations also passing distance-interval check |
| --- | ---: | ---: |
| 0m | 0 | 0 |
| 1m | 3 | 3 |
| 2m | 3 | 3 |
| 3.5m (experimental default) | 3 | 1 |
| 5m | 3 | 0 |

All three are track476 at morning109f-7 indices103,107,126; exact video review
confirms a real red lamp, not a taillight. The non-monotonic last column comes
from retaining the conservative distance rule: the ENTIRE supported interval
must lie within25-140m. Wider lateral allowance admits farther hypotheses too.
At3.5m, the intervals are approximately50-158,50-152 and50-134m respectively.
These are size/height-based hypotheses, NOT measured distances. Do not trim them
to140m and then claim reduced uncertainty. The default is not selected by
maximizing this last column; distance uncertainty is a separate unresolved limit.
The farther-left real red in109f-6 remains outside even the5m allowance.

None of those three passes the separate strict color/quality/state requirement;
the six strict red observations elsewhere still lack trustworthy boundaries.
Preparation entries remain zero. This corrects lateral candidate coverage; it
does not demonstrate earlier braking or improved driving. No ONNX weights,
detector color thresholds, stopped-red veto or acceleration limits are changed.

Negative checks: one green candidate passes geometry at every width; exact
109f-2index859 review shows a real green signal. A reviewed night109e-3index728
red candidate is a vehicle rear lamp. Morning109f-4index538 is an amber warning
signal, not established red-stop ground truth (a still does not establish its
flashing sequence). Both remain rejected for missing reliable geometry, so this
is NOT proof that the lateral margin itself rejects such objects. There is no
exhaustive annotated background/sign/branch false-positive rate in this screen.
Synthetic tests additionally check low rear lamps, far lateral objects, original
source conflict, expanded bounds and road clipping. **193 focused tests pass**.
No new camera inference or vehicle-response replay was performed. Default OFF;
no vehicle connection, enable or deployment. Private reproduction and review
frames: `.analysis/archive/2026-10-12-signal-lateral-margin/`.


### Range audit: outside the 3.5m allowance and close signals

This is a read-only audit of the same13 retained night/morning video segments,
3348 result records and fresh full-rlog road/lane/calibration context. All186
fresh measured housings were included:94 red,86 green,6 unknown. No observer
rerun or control threshold change was made. First failing geometry reason:
176 unreliable boundary pairs,6 lateral exclusions,2 intervals extending beyond
140m,2 accepted (one red and one green). Counts are observations, not lights.
Core-generated/template boxes are not measured housings and are excluded from
metric inference. These gates belong to moving preparation, not camera display.

YES, an actual red lamp lies outside the expanded projected lane region.
Morning109f-6 indices737,741,746,764,773,783 show the same overhead signal to the
left of the approaching car. Inner lanes satisfy the current confidence rule.
For height-compatible size/distance hypotheses, at least6.3-6.8m EXTRA allowance
beyond the left lane boundary would be needed;3.5m and5m both miss it. This is
modeled geometry, not surveyed lateral placement or proof of ego-signal ownership.
All six have a blinker on; the car subsequently turns right. The separate turn
exclusion would therefore prevent preparation even if the lateral gate widened.
Do not promote a wider margin just to make this signal trigger braking.

Close signals are detected but can fail preparation for other reasons. All35
measured boxes at least50px wide first fail boundary confidence. Width is a
review-selection heuristic, NOT a verified distance label. Exact-frame109f-8
index614 shows two large overhead green signals while driving nearly straight
(38.3km/h, steering0.7deg, no blinker). Inner probabilities0.420/0.137 and road
stds1.995/1.348 fail. The center housing spans105px: assumed0.7-2.5m width implies
7.6-27.2m forward range. If confidence is bypassed FOR DIAGNOSTICS ONLY, its
nominal lateral location fits within the lane and its supported interval still
includes distances below the25m preparation minimum. Thus widening the lane
allowance does not solve this close-signal exclusion. No actual distance was
measured, and the confidence bypass is not part of production code.

Exact109f-10index24 shows nearby red housings while turning (steering-109.8deg,
blinker on). Boundaries are unreliable (inner probabilities0.006/0.011). Depending
on uncertain distance assumptions some nominal positions fit and others exceed
3.5m; do not claim a reliable close-signal lateral miss. Distinguish missing road
geometry, distance uncertainty, turning and actual detector failure. Review
frames/context/reproduction: `.analysis/archive/2026-10-12-signal-range-audit/`.

### Integration and one user-facing mode selection

The user superseded the brief trial-only correction and requested the entire
signal-shadow contents on carrot-wip, with all future work linear on wip. Commit
e4c9730510 integrates the trial through5ce8faa735 as a SINGLE-PARENT commit; no
historical rewrite or merge commit. Existing wip radar/MPC/font fixes remain.
Retrained Mountain Dew8d736683 and its pinned runtime are included by that explicit
full-branch request; internal/DM and Jetson selections remain unchanged. The old
local trial ref is retained but will not receive parallel changes. The remote
carrot-signal-shadow branch was subsequently deleted at the user's request after
verifying its model/runtime and signal contents on carrot-wip. Do not recreate
the remote branch; continue with single-parent commits on carrot-wip. Integration checks:
247 Python tests and the later combined Web checks pass; no device deployment.

`SignalAssistMode` is a persistent integer default0, exposed in Carrot Web's
Traffic Light menu and searchable by name. Native Params registration is included;
after pulling, restart/build before selecting it. Subsequent mode changes are live.

- 0: auxiliary observer, camera overlays, helper alerts, departure veto and moving
  preparation OFF. Original driving-model signal logic is independent.
- 1: signal recognition/display; stopped-red departure veto with its existing
  once-per-blocked-departure audioRefuse warning. Moving preparation can evaluate
  and announce a qualified red using trafficSignChanged, but cannot change
  preparation xState or cap acceleration.
- 2: all mode1 features plus existing e2eStopPrepare and gradual acceleration cap
  toward-0.5m/s2 (0.25m/s3 ramp), with unchanged lead/pose/color/geometry/freshness,
  driver-override, speed and model-stop handover conditions. It is conditional
  assistance, not braking on every red-shaped box or permission to depart.

The selection supersedes production enabled/tracking/display/start/prepare files;
stale old enable files cannot bypass0. Retired moving assist_enabled cannot turn
on its older red override. Backend configuration files still select observation
implementation in1. Mode2 uses the CPU measured-housing tracker even with an eGPU
because preparation consumes that evidence contract; the external driving model
continues independently and DM GPU protections remain. This is not a new GPU
performance claim. Missing/unreadable/unregistered values remain OFF.

Selection is polled every0.5s; consumer/producer polling may take about1s and
observer process startup can take longer. Enabling discards observations captured
before activation. Preparation mode changes discard pending confirmation and
require new observations. Mode1<->2 retains an already latched stopped-red veto;
0 removes that helper veto, without issuing a go command. Camera display retains
its existing bounded visual memory only while enabled. Mode/control_active are
logged separately so a mode1 observation is not mistaken for actuation.

Validation includes mode0/1/2 planner+final acceleration checks,2->1->0 cap removal,
OFF->ON freshness, red-veto clearing, old-flag override, backend selection and
catalog/native-key registration. Focused signal suite275 tests passes on Windows
with native Params storage substituted;25 Wiki-generator tests and250 Web/eGPU
UI tests pass. The camera validity tests still use real Cap'n Proto messages.
Native rebuild, observer restart, hardware audio and actual vehicle response are
not validated by these desktop tests. The retained log screen still has ZERO
fully-qualified preparation entries; adding mode2 does not change that result.
