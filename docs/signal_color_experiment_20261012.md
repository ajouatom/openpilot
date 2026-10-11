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
