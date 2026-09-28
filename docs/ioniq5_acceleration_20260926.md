# Ioniq 5 acceleration oscillation, 2026-09-26

## Scope and evidence

Read-only investigation of HYUNDAI_IONIQ_5_PE, route segment
`00000fc6--8279a112c2--6`, incident commit `1b110e8c` on the private
Jetlink experiment. No controller, settings, radar, or model changes were made.
The findings concern shared longitudinal logic at that commit, not proof of a
Jetlink-specific regression. Raw captures and settings remain in the ignored
local analysis archive.

The full cereal schema decoded 5,994 carState, 5,999 carControl, and 1,199 each
of longitudinalPlan, radarState, modelV2 and livePose. Times below are seconds
from the first carState in the segment, not initData or route boot time.

## Observed mechanism

The strongest acceleration oscillation occurs at approximately 5–12 seconds.
There is no accelerator/brake input in 4–14 seconds. Lead track 32 remains
selected. The lead's estimated acceleration itself rises and falls; this
investigation does not independently establish its physical acceleration.

| Segment time | Basic MPC target | Target after preview | Controller request | Measured acceleration |
| --- | ---: | ---: | ---: | ---: |
| 6.00 s | 1.94 | 1.94 | 1.92 | 1.52 |
| 6.50 s | 1.67 | 0.85 | 0.93 | 1.91 |
| 7.00 s | 1.27 | 0.28 | 0.31 | 1.26 |
| 8.50 s | 2.15 | 2.15 | 2.50 | 0.73 |
| 10.50 s | 1.61 | 0.53 | 0.90 | 2.30 |
| 11.00 s | 1.14 | -0.03 | 0.15 | 1.61 |

All acceleration values are m/s², sampled from the nearest service message.
These are time-aligned observations, not a counterfactual vehicle replay.

1. Automatic driving mode enters Safe at 0.256 s and returns to Normal at
   5.263 s. Both September 26 settings snapshots (08:35:46 and 09:26:20)
   specify automatic mode, gap-1 response level 5, and LongActuatorDelay=30
   (0.30 s). The recorded personality remains aggressive. Safe caps response
   at level 3; Normal allows level 5. The recorded acceleration-change cost
   falls from 130 to 10 at the Normal transition. Level 5 also lowers the jerk
   cost to 15% and has no boost-entry ramp.
2. The preview signal is lead acceleration minus measured ego acceleration,
   with a 0.10 m/s² deadband. Consequently, a still-accelerating lead can
   request early acceleration reduction when ego acceleration is stronger.
   At 6.5 s the lead estimate is +0.16 and ego +1.91 m/s²; preview has reached
   1.36 s. The effective action horizon can grow from 0.35 to 1.85 s.
   While the basic target is positive, Normal has no maximum reduction delta;
   only the -0.03 m/s² floor applies. Maximum observed preview reduction in
   4–14 s is 1.234 m/s² at 10.805 s.
3. Preview changes acceleration after MPC, but vTargetNow remains the original
   MPC speed. The Hyundai speed-error P term can therefore request additional
   acceleration while preview has lowered feedforward. The P term reaches
   +1.113 m/s² in 4–14 s; the total request saturates at +2.5 m/s² during the
   second acceleration pulse. This is proportional correction, not integral
   windup: Hyundai Ki is zero.
4. Actual SCC_CONTROL output adds another temporal difference. At 8.0 s,
   transmitted aReqRaw is +2.17 while jerk-limited aReqValue is +1.05 m/s².
   At 10.5 s these are +0.96 and +1.86. The changing request is confirmed in
   outgoing CAN, not merely an incorrect UI or carOutput display. These
   values do not alone establish which SCC field the vehicle prioritizes.

## Verification and limits

The incident commit's preview function, applied to all 1,199 logged MPC
trajectories and preview horizons, reconstructs published aTarget with maximum
absolute error 0.0000108 m/s². This directly verifies the source of the large
post-MPC reductions. It does not simulate alternate vehicle motion or prove
the outcome of disabling preview or lowering the response setting.

There are no invalid modelV2/livePose/longitudinalPlan/carState/carControl or
radarState messages, no model frame-ID skips, and no failed livePose
inputsOK/sensorsOK/posenetOK flags. Road/wide SOF maximum gaps are
59.185/59.184 ms. CAN timeout and ACC-fault flags remain false. The 5–12 s
oscillation has no evidence of model interruption or lead-ID switching.

Later intervals must be separated: accelerator override occurs at
36.237–41.120 s and 53.442–54.792 s; brake input begins at 58.592 s.

The evidence supports an interaction between aggressive positive lead response,
post-MPC relative-acceleration preview, speed-error feedback and SCC command
ramping. It does not isolate every contributor's closed-loop effect or prove
a radar-estimation fault. A correction should evaluate coherent acceleration,
speed and jerk targets plus response-mode transitions, retaining lead-distance,
braking and validity constraints. It requires controlled comparison and vehicle
validation before claiming improved comfort; no mitigation was deployed here.

## Offline candidate screening (same-day follow-up)

The user authorized candidate development and comparison while keeping current
vehicle behavior unchanged, with no deployment unless evidence supported it.
Three executable candidates were evaluated only in local analysis code:

- `velocity_consistency`: accumulate the negative post-MPC acceleration
  correction into a speed-reference offset, reconciled with a 2 s time constant.
  This is a hypothesis, not a complete coherent replanning implementation.
- `positive_slew`: limit increasing positive controller requests to 1 m/s³;
  leave zero/negative requests and all command reductions immediate. An initial
  diagnostic version also limited negative-command release; it was corrected
  before the final sweep. Final results refer to the positive-only version.
- `boost_entry`: ramp entry to maximum level-5 MPC response over 0.4 s, retaining
  immediate release, unchanged levels 0–4, unchanged eligibility and the same
  eventual maximum-response weights.

**Decision: none is approved for runtime or vehicle deployment.** Production
control files, settings, and vehicle processes remain unchanged. No experiment
branch was created. These are rejected/deferred candidate experiments, not an
implemented fix.

### Recorded-input screening

On the incident's fixed recorded inputs (4–14 s), the speed-offset candidate
creates a minimum -0.886 m/s² command where the baseline remains positive;
negative commands last 2.406 s. Its command total variation increases from
15.831 to 17.219 m/s². Positive-only rise limiting lowers total variation to
11.137, but leaves the largest half-second command drop unchanged at
-1.650 m/s². A smoother command plot alone is therefore insufficient.

Ten historical segments, including earlier preview release/handoff cases, add
39,189 active controller samples. The speed-offset candidate increases total
variation in nine segments and creates 11.148 s of new negative commands.
Positive-only rise limiting increases total variation in none and introduces
no new negative commands. Historical controller inputs and recorded lead
assignments remain fixed; this does not rerun their original planners or
predict alternate vehicle motion. The MPC-entry candidate cannot be evaluated
by overlaying output on fixed old MPC solutions.

### Diagnostic feedback simulation

A local nonlinear least-squares backend runs the production Python MPC update
wiring and its cost/soft-constraint equations. It is **not the native acados
SQP_RTI solver**: it uses exact linear dynamics and full least-squares iterations.
The model includes ego-speed feedback, relative distance, production preview
and response helpers, a simplified Hyundai P/feedforward command, ordinary SCC
jerk ramping and a delayed first-order acceleration response. It omits complete
stopping/interlock behavior, navigation, perception uncertainty and the full
vehicle runtime. Incident inputs retain logged lead motion/acceleration/tau,
mode and TF schedule; the latter are held fixed rather than rerunning automatic
mode/TF detection under alternate ego motion. Synthetic cases use a fixed TF.

The effective SCC-value to logged-aEgo fit uses 4–18 s for fitting and 18–35 s
for holdout. The best tested delay is 0.40 s with time constant 0.093 s, gain
0.896 and bias -0.064 m/s². Training/holdout acceleration RMSE is
0.210/0.130 m/s². This includes measured-acceleration filtering and must not be
interpreted as independently calibrated physical actuator latency.

The final sweep contains 144 runs: baseline plus three candidates, four
scenarios (recorded lead motion, repeated acceleration, lead braking, cut-in),
and nine delay/time-constant combinations (0.2/0.4/0.6 s and 0.05/0.15/0.30 s).
All final numerical solves report convergence. A finite-difference check of
12 solver Jacobians has maximum absolute error 3.12e-9. Neither numerical
convergence nor derivative correctness establishes native-solver equivalence.

| Candidate | Comparisons with >5% higher RMS acceleration jerk | Comparisons with >5% higher peak acceleration jerk | Comparisons with >5% higher lead-speed tracking RMS error |
| --- | ---: | ---: | ---: |
| Speed offset | 18 / 36 | 10 / 36 | 18 / 36 |
| Positive-only rise limit | 2 / 36 | 4 / 36 | 18 / 36 |
| Maximum-response entry ramp | 0 / 36 | 2 / 36 | 11 / 36 |

The 5% cutoffs summarize relative changes; they are not validated comfort or
safety acceptance thresholds. For example, the entry-ramp candidate raises
peak acceleration jerk from 3.527 to 3.789 m/s³ in the incident-input scenario
with 0.20 s delay and 0.05 s response time constant. This is an adverse signal
in the diagnostic model, not proof that the actual vehicle would do the same.

The substitute baseline at 0.40 s delay / 0.15 s response has speed RMSE
0.302 m/s and acceleration RMSE 0.377 m/s² against the recorded incident. That
remaining error and the omitted behaviors preclude certifying small candidate
improvements or declaring actual stopping-distance equivalence. No tested
candidate reduces minimum gap by more than 0.1 m or delays the measured -0.5
m/s² braking crossing by more than 0.05 s in this model, but those limited
comparisons are **not** evidence of safety under untested conditions.

### Regression scope and next prerequisite

The existing preview/preview-release/gap-recovery suite passes 313 tests with
production code unchanged. An additional 34,560 local entry-ramp comparisons
verify helper-level parity for levels 0–4, no stronger boost than baseline,
maximum-response endpoint parity and immediate release. These do not replace
native MPC or vehicle validation.

The initial standard pytest invocation could not load Windows-unavailable
`params_pyx`; the successful focused run used `--confcutdir` to avoid the native
root fixture. The repository's native MPC extension is also absent on this
Windows host. A next candidate needs a native Linux/acados replay environment,
matching source and settings, preceding-state warmup, and a broader vehicle
response/perception uncertainty envelope before any runtime adoption. No native
replay success or actual vehicle improvement is claimed here.

Scripts, fixed-input reports, fitted model, all simulated trajectories, candidate
checks and a comparison plot are retained in the private local archive under
`.analysis/archive/2026-09-26/accel-candidates/`. See its INDEX.md to reproduce
and distinguish recorded evidence from diagnostic simulation.
