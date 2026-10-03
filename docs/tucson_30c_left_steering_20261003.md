# Tucson straight-road left steering near a vehicle transporter

Follow-up: the user requested a correction after the initial analysis.
The speed-mismatch fallback is now implemented; see the implementation and
replay section below. Earlier no-change statements describe the initial
investigation, not the final repository state.

## Scope and finding

User-reported left steering on a visually straight road, with a screenshot at
8.14 s of `0000030c--adf522a321--4`, device `cb9c2a66064c307d`.
The original full rlog and qcamera video were read from the NAS route directory
for `HYUNDAI_TUCSON_4TH_GEN`. Recorded commit is
`6be416242032ff04c15bf52872927094e8e3949f`, carrot-wip, dirty=false, C4/mici.

The left steering is real in the recorded command and wheel angle. Its onset
precedes the screenshot, principally at about 6.4–7.5 s. The vehicle is using
torque control and active lane-line planning. This is not the angle-control
handover-mode-3 path.

The strongest anomaly is the model velocity trajectory dropping from roughly
30 m/s to 10 m/s while measured vehicle speed remains about 29 m/s (104 km/h).
The lane-line MPC uses that model velocity trajectory. Its leftward curvature
is substantially larger and persists longer than the model's separate action
curvature. The logs support investigating the combination of model trajectory
change and lane-planner response, rather than assigning the entire event to a
large left-turn action directly from the model.

The adjacent transporter casts a moving shadow across the ego lane during the
event. Shadow, vehicle appearance/occlusion, and visual-motion effects cannot
be isolated from this single pass. Shadow involvement is plausible; a shadow
mistaken specifically for a lane line is not established.

## Timing and signs

Times below are seconds from the first carState message, not boot-time
initData. Video frames are matched with qRoadEncodeIdx.timestampSof and
segmentId. Model outputs are published after image exposure, so a displayed
frame and a contemporaneous output are not simultaneous sensor observations.
Numbers in the table use the nearest available publication; exact extrema
below use full-rate carState/control data.

Wheel angle and reported steering command torque are left-positive. Model
position y and model/control curvature are right-positive. Consequently,
negative recorded curvature here requests a left turn. The user-facing plot
negates curvature to put left-positive quantities on the same visual convention.

| Approx. time | Measured speed | Model velocity x[0] | Model action curvature /m | Control desired curvature /m | Actual wheel angle |
| --- | ---: | ---: | ---: | ---: | ---: |
| 5.5 s | 104.5 km/h | 30.90 m/s | -0.000098 | -0.000061 | +1.8° |
| 6.5 s | 104.6 km/h | 11.82 m/s | -0.000236 | -0.000380 | +1.6° |
| 7.0 s | 104.5 km/h | 9.93 m/s | +0.000021 | -0.000857 | +5.2° |
| 7.2 s | approximately 104.5 km/h | 10.26 m/s | +0.000129 | -0.001053 | +5.8° |
| 7.5 s | 104.5 km/h | 10.38 m/s | +0.000801 | -0.000542 | +7.0° |
| 8.14 s | approximately 104 km/h | 10.19 m/s | +0.002617 | +0.002641 | approximately -3.5° |

- Maximum left control curvature magnitude is 0.00105299/m at 7.174 s,
  versus maximum left model-action magnitude 0.00023607/m at 6.504 s.
  These are different-time maxima, not a measured fixed amplifier gain.
- The reported applied command peaks at +147 CAN units at 7.228 s.
  Actual wheel angle reaches +7.0° at 7.492 s, versus approximately +1.8°
  before the disturbance. The pre-event angle is a baseline, not a zero-offset
  calibration or a physical road-wheel angle.
- Strong negative column torque appears later. SteeringPressed first becomes
  true at 7.693 s, with torque -561 and reported applied command still +58.
  The wheel then reaches -12.6° at 7.852 s; column torque reaches -803 at
  7.865 s. This is consistent with a strong rightward corrective intervention.
  Sensor torque alone does not prove intent or precisely separate driver force
  from road/EPS reaction.
- The controller's limited command unwinds through zero near 8 s. The sharp
  rightward wheel excursion is not evidence that it commanded that same abrupt
  wheel motion. The screenshot is already in the rightward correction phase.

## Why lane planning matters

`carState.useLaneLineSpeed` is 1 throughout all 6,000 samples.
`controlsState.activeLaneLine` is true until 8.01953 s, false until 9.11751 s,
then true. Both blinkers are off and model lane-change state is off throughout
4–10 s. Logged lane offset is 0.0 cm. InitData contains UseLaneLineSpeed=0;
that startup snapshot must not override the actual recorded runtime fields.
The planner refreshes its value from carState.

At the recorded commit, lateral_planner.py uses the norm of modelV2.velocity
as v_plan, uses it in MPC dynamics, and converts MPC yaw rate to curvature
using that velocity. Lane paths are also sampled on the model time trajectory.
controls/controlsd.py selects this MPC result while activeLaneLine is true,
with lag adjustment and smoothing, rather than directly selecting model action.

At 5.5 s, the MPC predicts about 10.52 m of travel over the configured 0.34 s
delay-plus-smoothing interval; at 7.0 s it predicts only 3.34 m, while the
vehicle still travels about 9.87 m in 0.34 s. The model position at one second
also shortens from 30.97 m to 9.73 m. These are internally consistent shortened
model trajectories, not a measured drop in vehicle speed.

At 7.0 s, the lane-derived input lies only approximately 4.7 cm left at its
origin and 6.5 cm left at 10 m; MPC initial curvature is -0.001371/m and final
control curvature is -0.000857/m. At 7.2 s the lane input has moved slightly
right (+2.8 cm at origin), but MPC initial curvature is still -0.001313/m and
final control curvature is -0.001053/m. The separate model action has already
turned right. This identifies the lane-planning/control history as relevant
to both amplification and delayed reversal.

This is source-and-log reconstruction, not a same-input MPC counterfactual.
It does not quantify how much is caused by velocity, lane geometry, MPC state,
or smoothing, or prove that replacing model velocity alone fixes the event.
The lowered model speed is a planned model output; calling it an erroneous
measurement of actual speed would be inaccurate.

## Existing automatic laneless fallback

The user correctly recalled having added an automatic fallback for model
deceleration. At this recorded commit and in the current working source,
`lateral_planner.py:117` checks:

```python
if md.velocity.x[-1] < md.velocity.x[0] * 0.7:
  self.lanemode_possible_count = 0
  self.laneless_only = True
```

This compares the end and start of one predicted trajectory. It neither
compares model speed with measured carState.vEgo nor detects a drop between
successive model frames. At 7.0 s, the start is 9.93 m/s and end 12.09 m/s,
so the condition is false despite measured speed of about 29.04 m/s.
All 1,201 model messages in this segment fail to trigger this condition.
Across 6–8.2 s, the end/start ratio ranges from 0.994 to 1.478, always above
0.7. The low model velocity therefore does not force laneless operation.

The observed fallback has separate evidence: lane_planner_2.py reduces lane
probabilities using lane-line standard deviation and width consistency. At
model time 8.009369 s, reconstructed effective probabilities are 0.257001
left and zero right. The resulting d_prob is below 0.3, resetting the lane
confidence counter. lateralPlan.useLaneLines becomes false at 8.016472 s
and controlsState.activeLaneLine at 8.019530 s. The one-second confidence
reacquisition requirement is consistent with return at 9.117514 s. Thus this
transition matches the lane-confidence fallback, not the model-speed gate.

The speed guard exists and addresses a different condition. Any follow-up
should explicitly consider whole-trajectory speed mismatch, rather than
assuming the existing relative-deceleration condition already covers it.

## Shadow assessment and exclusions

Synchronized images show the transporter being passed on the right, with
its shadow covering increasing portions of the lane during the speed-output
drop and steering disturbance. The model's raw position path has only modest
left displacement initially (about -0.15 m at 30 m around 6.5 s), and shifts
right during the subsequent correction. A visibly rightward path at the
screenshot therefore does not contradict the preceding left steering.

A large, sudden left bend in the model's original action is not observed.
Nor is there enough evidence to claim that the model followed the shadow
edge. The vehicle itself, partial lane occlusion and shadow change together.
The low-resolution qcamera frames are not the complete original model inputs;
they cannot establish a specific internal model perception failure.

For 4–10 s, carState, modelV2, lateralPlan, carControl, controlsState and livePose
publications are valid; CAN is valid and neither steering fault flag is set.
Lateral control stays active. Camera/model publication intervals reach about
70 ms, so these checks are not a claim of perfectly uniform scheduling.
Calibration remains calibrated and learned steering ratio stays near 13.76;
there is no sudden parameter step comparable to the model-speed collapse.
No fault event explains the initial leftward command. Driver override follows
the leftward buildup, rather than preceding it.

## Next investigation and retained evidence

Prioritize an offline same-input comparison of the existing lane planner with
the original model action, and controlled alternatives for a model-trajectory
speed mismatch, preserving coordinate/time consistency. A change to speed
alone without consistent path timing is not a validated correction. Also
inspect MPC-state persistence, mode transitions and the existing torque slew
limits during corrective driver input. No controller changes or parameter
changes were made in this investigation.

Private scripts, numeric extracts, synchronized images and plots are retained
in `.analysis/archive/2026-10-03/tucson-30c-steering/`. Raw rlog/video remain on
the original NAS; captures and settings are not part of this tracked document.

## Implemented correction and same-input verification

The concrete software defect is that the lane-mode suitability guard only
compared two values inside the same model trajectory. It allowed a trajectory
at approximately one third of measured speed to drive lane MPC at highway
speed. The small lane-centering correction was consequently processed over
a much shorter distance/time trajectory, with persistent MPC state and
downstream smoothing prolonging the leftward target after the model's action
had turned right. This is a control-path defect irrespective of which visual
feature first changed the model output.

`LaneModelSpeedGuard` additionally requires the model's starting longitudinal
speed to be at least 70% of measured carState.vEgo. It reuses the existing
0.7 ratio, preserves the original end/start condition, and preserves the
existing >20 consecutive good model frames before re-enabling lane mode.
Nonfinite/negative speed inputs and missing model speed arrays cannot retain
eligibility. The new check is independent of the half-second settings refresh.
The existing laneless action selection, smoothing and curvature/actuator
limits remain unchanged. There is no new user parameter and no substitution
of measured speed into an unchanged model trajectory.

Validation used the recorded full segment and the incident source version:

1. The original lane planner and lateral planner classes were executed with
   recorded model/car inputs, recorded tuning, initialized pre-segment lane
   eligibility, and an independent small-angle MPC backend. This backend uses
   the production horizon, dynamics linearization, costs and carried state;
   it is not the native acados binary. Original mode decisions match all
   1,200 lateralPlan publications. Over 5.5–7.65 s, maximum reconstructed
   initial-curvature error is 0.000000391/m; maximum input-path discrepancy is
   0.000101 m. Over 4–10 s, maximum curvature error is 0.000001353/m.
2. A separate one-step MPC check seeded from each recorded initial state has
   maximum next-state curvature error 0.000000193/m and maximum heading error
   0.000004738 rad in the checked 5–7.7 s window. Resampling the same spatial
   lane input consistently at measured speed reduces the leftward next-state
   curvature at 7 s from approximately -0.001420/m to -0.001005/m even while
   retaining the already-leftward initial curvature. This isolates a speed/
   horizon contribution; that alternative resampling is diagnostic only and
   is not the shipped change.
3. Both original and modified planner outputs were passed through the existing
   lag adjustment, 0.11/0.10 s smoothing, and curvature limits at recorded
   control timestamps. Original target reconstruction has maximum error
   0.000011067/m over 4–10 s. Model input and publication timing are retained;
   asynchronous sampling prevents bit-exact control reconstruction.
4. The new guard trips on the 6.308573 s model frame. Modified lateralPlan
   leaves lane mode at 6.315849 s; target replay changes source at 6.330105 s.
   The initial leftward target peak over 5.5–7.65 s decreases from recorded
   0.001052992/m to 0.000218774/m, approximately 79.2%. At 7.2 s, the old target
   is -0.001052961/m (left), while the modified target is +0.000024538/m (right).
5. Continuous good-speed confirmation permits lane-mode return at 9.711110 s
   planner publication, approximately 9.713894 s in control replay. The modified
   planner is rerun throughout fallback and return; its re-entry MPC output is
   not copied from the original lane-enabled recording.
6. Twenty-six focused tests cover normal-profile equivalence, both speed gates,
   ratio boundaries, invalid/missing speed inputs, interrupted recovery,
   explicit laneless selection and the production planner's routing. The
   routing tests substitute the native numerical/IPC dependencies; the separate
   numerical reconstruction above checks the incident's MPC behavior. The
   focused tests also run in the `Lane-mode model speed fallback` CI job.

The 79.2% figure is a target-curvature result on fixed recorded inputs, not a
measured reduction in wheel motion or lane displacement. After changed steering,
the real scene, model outputs and driver response would differ. This revision
does not prove that the shadow itself was misclassified, nor does it eliminate
all model-perception failures. No device was reflashed or driven for this test.

Implementation replay scripts and extracts are retained privately under
`.analysis/archive/2026-10-03/tucson-lane-fix/`. User guides and the catalog's
localized Wiki descriptions explain the temporary fallback and recovery.
