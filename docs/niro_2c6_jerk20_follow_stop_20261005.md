# Niro 2c6: continued low-speed approach after the jerk-cost trial

## Conclusion

The reported slow final approach is present in the new log. Jerk cost 20 is
deployed, but this drive is not a controlled before/after comparison of comfort.
One important difference from the earlier investigated tails is now identifiable:
the existing opening-gap recovery logic adds comfort headroom during braking,
then stops recovering that headroom as the lead reaches low speed. This contributes
to a more distant MPC following reference during the middle/late approach.

The very last crawl has an additional, separate tracking discrepancy: actual
deceleration is much weaker than the negative acceleration command until the
explicit stopping request. Extra headroom alone does not explain the complete
tail or the final distance below the configured standstill reference.

Analysis only. The user-authorized J_EGO_COST=20 trial remains in place; no new
control change, setting change, commit, push or NAS deployment was made here.

## Source and configuration

- KIA_NIRO_EV, dongle 88baf851efe58d6c, route `000002c6--d04b5cd552`, segments 4–5.
- Full cereal rlogs and qcamera videos were read from the prescribed NAS route.
- initData reports `7432ac9b5c0cac7b92fdf8393c5dda4d2de9a99b`, carrot-wip,
  dirty=false, mici. That revision sets J_EGO_COST=20; the recorded aggressive
  personality gives effective jerk weight 10 when response reduction is inactive.
  There is no directly logged final jerk-cost-weight field.
- Response level 3, TF 0.90 s, Normal driving mode, automatic mode switching off,
  stopping distance 6 m, actuator delay 0.30 s, VEgoStopping 0.10 m/s,
  StoppingAccel -0.50 m/s², radar mode 2. Startup settings and the 2026-10-05
  08:55:28 snapshot agree. The relevant fields also match the 2026-10-04 16:22:22
  snapshot from the previous 2c5 drive. All W: branch/fingerprint directories
  were searched for this dongle; 08:55:28 was the newest snapshot found.
- CP.vEgoStopping remains 0.5, but the planner uses the saved VEgoStopping=0.10;
  those fields must not be conflated.

Time zero is segment 4 first carState at boot time 295.416201814 s, corresponding
to 2026-10-05 08:58:24.464866 KST. Segment 5 starts at relative 59.971 s.

All carState, carControl, controlsState, selfdriveState, radarState, liveTracks,
modelV2 and longitudinalPlan records examined are valid. LeadOne is continuously
front-radar track 32; leadTwo is absent. Control remains active with no gas/brake
input, no forceDecel, stock AEB/FCW, or soft hold. XState stays lead (0), and
navigation/turn speed limits are above the set speed. Video samples show the same
dark SUV slowing toward the signal queue; adjacent passing vehicles do not
correspond to a recorded lead identity change.

There are three isolated `selectionStale` fast-overlay fallbacks in 40–70 s,
at 48.792, 49.041 and 57.742 s. Published radar remains valid with the same lead;
these are not sustained lead loss or proof of the cause of the prolonged tail.
The plan source label sometimes says cruise because it is chosen by the current
obstacle minimum. The horizon can still be lead-constrained; the label does not
mean the lead is ignored or that the 80 km/h cruise setting commands crawling.

## Observed approach

The minimum actual aEgo is -2.341 m/s² at 45.301 s. Minimum target is -2.322 m/s²
at 44.739 s, and minimum initial planned jerk is -1.298 m/s³ at 44.140 s.
Lead aLeadK reaches -1.898 m/s² at 49.636 s; this is a radar-derived estimate,
not an independent measurement of the lead's physical braking.

The older events had different lead motion, initial gaps and speeds. Comparing
their higher peak braking directly with this event cannot isolate the benefit
of J20. The driver's report of reduced roughness is compatible with the intended
change but is not a controlled quantitative validation.

| Milestone | Relative time | Ego speed | Radar range |
| --- | ---: | ---: | ---: |
| Ego first below 10 km/h | 53.350 s | 9.98 km/h | 13.29 m |
| Lead first below 0.30 m/s in magnitude | 57.344 s | 5.65 km/h | 12.03 m |
| Ego first below 5 km/h | 58.269 s | 5.00 km/h | 10.74 m |
| Planner shouldStop | 65.632 s | 1.86 km/h | 3.81 m |
| SCC12 StopReq=1 transmitted | 65.641 s | approximately 1.86 km/h | approximately 3.81 m |
| CarState standstill | 66.730 s | filtered speed 0.38 km/h | 3.40 m |
| Filtered ego speed below 0.10 m/s | 66.751 s | 0.35 km/h | 3.40 m |

The 5 km/h-to-near-stop interval is 8.48 seconds; from 10 km/h it is 13.40 seconds.
Settled radar range is about 3.30 m (median over 70–90 s). The configured 6 m
reference is not the actual final distance. Neither radar range nor the setting
is a surveyed bumper-clearance measurement.

For context, previous 2c5 events took 5.65–6.34 seconds below 5 km/h and the 2c3
event took 6.86 seconds. This confirms continued occurrence in the uploaded drive;
it does not establish that increasing jerk cost caused the longer observed tail.

## Newly demonstrated comfort-headroom retention

After the stronger braking, ego becomes slower than the lead. Range increases
from 11.597 m at 47.192 s to 13.714 m at 55.293 s, a 2.117 m opening. The gap
recovery code intentionally captures part of that opening as extra comfort TF.

Production LeadGapState was replayed chronologically through the published plans.
The selected liveTracks frame is aligned using plan.liveTracksMonoTime; active
overlay kinematics use track 32, zero front-radar delay and the preceding measured
ego speed. Published radar is used on recorded overlay fallback. Other vehicle
motion remains unchanged. Unknown segment-entry history is initialized locally.

The reconstructed displayed reference matches the logged reference over 40–66 s
with mean absolute error 0.0178 m and maximum error 0.1468 m. This is reference-
layer replay, not a full native MPC or vehicle replay. It corroborates the active
headroom mechanism despite small input timing differences and unknown initial
history. It does not measure the headroom's counterfactual effect on acceleration.

| Time | Ego / lead speed, km/h | Actual range | Base reference estimate | Logged reference | Reconstructed extra TF |
| --- | ---: | ---: | ---: | ---: | ---: |
| 48 s | 28.77 / 30.98 | 11.96 m | 11.67 m | 11.68 m | 0.001 s |
| 53 s | 10.67 / 12.16 | 13.29 m | 8.45 m | 9.96 m | 0.496 s |
| 55 s | 7.79 / 7.44 | 13.71 m | 8.41 m | 10.33 m | 0.815 s |
| 58 s | 5.20 / approximately 0 | 11.10 m | 7.83 m | 9.02 m | 0.794 s |
| 60 s | 4.06 / approximately 0 | 8.42 m | 7.16 m | 7.98 m | 0.794 s |
| 64 s | 2.56 / approximately 0 | 4.75 m | 6.43 m | 6.77 m | 0.794 s |

In `longitudinal_gap_recovery.py`, recovery strength decreases as lead speed drops
from 5 to 0.3 m/s and becomes zero at/below 0.3 m/s. Thus the extra TF stops
recovering near 0.794 s after the lead stops. The extra *distance* is still speed
dependent: it shrinks from about 1.9 m near 55 s to about 0.34 m at 64 s, and the
final zero-speed reference returns to 6 m. Do not describe this as holding a fixed
10 m stop distance or changing the physical danger-zone TF from 0.9 to 1.7.

This is an intended comfort-reference behavior now visible in the incident,
unlike the earlier investigated late-tail samples where additional headroom was
approximately absent. It is a specific candidate for further MPC-reference
analysis, not proof that removing it alone cures the complaint.

## Final low-speed tracking discrepancy remains

At 62 s, actual speed is 3.15 km/h versus current planned target 2.48 km/h.
The planner target is -0.238 m/s² and controller request about -0.425 m/s²,
but measured aEgo is only -0.053 m/s². At 64 s, actual/target speed are
2.56/1.55 km/h; request is about -0.600 versus measured -0.151 m/s².
The controller is already requesting more braking to reduce a positive speed
error; it is not asking the car to hold its measured creeping speed.

Median values around the recorded shouldStop edge:

| Interval | SCC12 requested acceleration | Actual aEgo |
| --- | ---: | ---: |
| -1.5 to -0.2 s before edge | -0.600 m/s² | -0.130 m/s² |
| +0.35 to +0.8 s after edge | -0.540 m/s² | -0.478 m/s² |

Actual braking becomes stronger after stopping mode despite the slightly weaker
negative request. This repeats the earlier low-speed response observation, but
does not isolate StopReq, comfort bands, brake blending, grade or ECU behavior
as a sole cause. Final range below the target also cannot be explained solely
by an overly large comfort gap.

## Lead prediction is mixed, not a uniformly pessimistic forecast

Using production extrapolation with published radar samples, observed future lead
travel is approximated by integrated measured ego speed plus radar-range change.
Track identity is continuous. These are prediction diagnostics, not a full fast-
overlay replay or independently surveyed lead trajectory.

| Snapshot | Predicted next 4 s travel | Observed reconstruction |
| --- | ---: | ---: |
| 43.996 s | 40.71 m | 42.44 m |
| 44.996 s | 33.23 m | 37.62 m |
| 45.995 s | 28.71 m | 33.02 m |
| 46.995 s | 30.27 m | 28.46 m |
| 49.992 s | 10.19 m | 16.04 m |
| 53.987 s | 8.79 m | 5.17 m |

Some moments predict less lead travel, others more. A blanket faster tau recovery
is not justified by the pessimistic snapshots alone. In the final crawl the
published tau has already returned to its ordinary value around 1.5; persistent
small tau does not explain that entire tail.

## Next investigation supported by this log

Retain the user's J20 driving trial while comparing the additional comfort-gap
capture/recovery policy specifically during a shared low-speed stop approach.
This stays within the MPC reference path rather than adding a conditional stop
override or relaxing physical danger penalties. It should be isolated from the
separate low-speed command-tracking issue. A baseline-matching MPC comparison
would be needed to quantify the acceleration/creep effect of reference changes.
No such counterfactual vehicle result or production fix is claimed here.

## Follow-up: minimum 30% recovery strength

The proposed 30% is interpreted as a lower bound on recovery strength, not
instant removal of 30% of distance. There are two separate mechanisms:
filtered measured relative speed above 0.2 m/s selects charge/hold rather than
release; after that condition ends, lead absolute speed scales release down to
zero at 0.3 m/s. Therefore a strength floor alone cannot release headroom while
the measured gap is opening. The earlier plain-language explanation mentioning
only lead speed omitted this first condition.

Replayed 1,334 archived input frames before standstill. Raw relative speed was
recovered by inverting the stored first-order filtered recurrence, using the
acquisition value for the first frame. Production baseline extra/reservoir state
matches the archived reconstruction exactly (maximum difference 0.0). This is
agreement with the earlier reconstruction, not independent validation against
an internal logged gap state.

Compared unchanged code with (a) a global strength floor of 0.30 and (b) the
same floor only in the release branch. Both candidates match each other for
this event because the floor becomes relevant after the opening interval.
Generally the global version also speeds output following during capture and
can increase additional headroom sooner; a release-only experiment isolates
the proposed removal of the low-speed freeze.

| Approximate segment time | Existing additional reference | Release-only 30% floor |
| --- | ---: | ---: |
| 55 s | 1.916 m | 1.916 m |
| 58 s | 1.196 m | 1.012 m |
| 60 s | 0.824 m | 0.512 m |
| 62 s | 0.547 m | 0.234 m |
| 64 s | 0.343 m | 0.096 m |

These are additional comfort-reference metres on the same recorded planned
speed, not changed physical ranges. Baseline extra TF remains about 0.794 s;
the candidate decreases to 0.493 s at 60 s and 0.223 s at 64 s. Absolute extra
metres already decrease in baseline as planned speed decreases. A 30% floor
starts to change the rule below 6.156 km/h lead speed. At response level 3,
constant minimum strength gives k=0.3/s; the two-stage decay is continuous,
not 30% distance removal per second or per frame.

The floor is a bounded reference-layer candidate, but leaves the initial
11.6-to-13.7 m opening and low-speed capture untouched on these recorded inputs.
To address that phase, separately compare limiting additional comfort capture
during shared deceleration; do not assume every opening gap is deliberate
headroom or immediately force its closure. No acceleration target, stopping
time, physical clearance improvement or closed-loop safety result is established
by this replay. Keep physical lead obstacles and danger constraints unchanged.
Production code has not been modified. Reproduction and results are archived
under `.analysis/archive/2026-10-05/niro-recovery30/`.

## Follow-up: distance ratio overrides comfort-headroom hold

The user proposed ignoring speed as the recovery permission and removing the
hold condition when range exceeds 1.5 times the TF distance. Compared two
explicit definitions, both excluding extra headroom: (1) original base desired
distance, floored at the existing 6 m stop distance; (2) TF-distance defined as
baseTF times planned ego speed plus the 6 m stop distance. The second definition
is used in the user-facing result below. Pure speed times TF alone tends to zero
at standstill and is unsuitable as a full physical following-distance reference.

In the experiment, the distance gate disables opening-gap capture/hold and
bypasses the lead-speed scaling of release, retaining the existing response-3
two-stage dynamics at full strength. It does not reset extra TF to zero, change
base TF or relax the physical lead obstacle or danger constraints. Removing
only opening hold would still leave release frozen at stopped lead speed.

Using the TF-plus-stop definition, the gate is active from 52.880 to 57.981 s
on the recorded inputs. The first active ratio is 1.511. The current additional
comfort reference changes as follows:

| Approximate time | Existing | Distance-gated recovery |
| --- | ---: | ---: |
| 54 s | 1.785 m | 1.099 m |
| 55 s | 1.916 m | 0.568 m |
| 56 s | 1.714 m | 0.255 m |
| 58 s | 1.196 m | 0.039 m |

With the original base desired distance instead, the gate activates earlier,
52.282 to 57.042 s, and 55 s extra reference is 0.300 m. A separate illustrative
1.5-entry/1.2-exit hysteresis variant of that definition remains active through
59.780 s; the exit threshold is not a selected or validated production value.
Hysteresis or smooth blending should be considered to prevent boundary chatter
and rapid reacquisition near a single threshold. Acquisition of a newly selected
lead still initializes the candidate in these prototypes.

Baseline extra/reservoir reconstruction again matches all 1,334 pre-standstill
archived frames exactly. These comparisons change the current comfort-reference
state on fixed recorded inputs only. No future MPC rollout, acceleration output,
vehicle plant, actual gap or stopping-time effect has been evaluated. Reducing
the comfort reference may permit stronger approach; retaining physical danger
constraints alone is not closed-loop validation. Production remains unchanged.
Scripts/results: `.analysis/archive/2026-10-05/niro-gap-ratio/`.

## Approved implementation

The user subsequently approved a smooth hold range: original conditions through
1.2D, progressively reduced hold/capture and restored recovery strength through
1.5D, and ordinary level-rate recovery beyond it. D is TF times MPC ego speed
plus configured stopping distance, excluding extra headroom. Implemented and
pushed as `be1a9442f0`; see `longitudinal_gap_hold_band_20261005.md` for the exact
rule, 299 focused tests and remaining vehicle-validation limits. Earlier
"production unchanged" notes describe the preceding analysis stages only.

Final fixed-input replay matches the archived baseline exactly for 1,334 frames;
additional reference at 55 s changes from 1.916 to 0.142 m, and at 58 s from
1.196 to 0.008 m. An additional 1,800-solve synthetic lagged-plant comparison
shows the intended faster approach can also increase later deceleration and
slightly undershoot the preferred gap. It is independent full-convergence OCP,
not native acados or a calibrated Niro plant. No physical stopping-time,
clearance or ride-quality result is established. Private scripts and selected
results: `.analysis/archive/2026-10-05/gap-hold-band/`.

Private scripts, selected metadata, timeline/gap reconstruction, forecast results
and visually inspected charts/contact sheet are retained under
`.analysis/archive/2026-10-05/niro-2c6/`. Original rlogs/videos remain on NAS;
reproducible decoded logs, video copies and dependency cache are removed locally.
