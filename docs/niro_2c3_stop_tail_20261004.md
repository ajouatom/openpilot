# Niro 2c3: braking-to-stop shape and prolonged low-speed approach

Follow-up candidate replay: `niro_tau_recovery_trial_20261004.md`. The tested
causal recovery rules did not improve aggregate 4-/6-second lead prediction
error and produced optimistic counterexamples. Prior hindsight prediction
errors alone do not justify implementation. Production remains unchanged.

## Conclusion and scope

The clarified complaint concerns the full speed profile: pronounced earlier
braking followed by a prolonged crawl to the final position. Necessary initial
braking does not by itself justify that tail. The earlier 2c5 observation that
range did not grow also does not disprove this complaint.

Route `000002c3--4363018d77`, segments 0–1, reproduces the prolonged tail.
Ego takes 6.86 seconds from first dropping below 5 km/h to below 0.1 m/s,
closing logged range from 8.95 to 3.90 m. The stronger braking phase removes
substantial closing speed; ego is still faster than the selected lead at the
representative samples throughout that phase. The evidence does not establish
that excessive initial braking created a large opening gap. It does establish
a long low-speed taper, weak actual deceleration relative to negative requests
late in the approach, and a late transition to explicit stopping.

No production code or vehicle settings changed. The user explicitly requested
analysis without forcing a speculative fix. Whether a different speed profile
can improve comfort while retaining clearance requires a separate controller
comparison and vehicle validation; it is not proved impossible or inevitable.

## Recorded configuration differs from the earlier upload

| Setting | 2c3, this upload | 2c5, earlier analyzed upload |
| --- | --- | --- |
| Response level | 4 | 3 |
| Runtime TF during event | 0.85 s | 0.90 s |
| StopDistanceCarrot | 6 m | 6 m |
| LongActuatorDelay | 0.20 s | 0.30 s |
| VEgoStopping | 0.10 m/s | 0.10 m/s |
| StoppingAccel | -1.0 m/s² | -0.5 m/s² |

Both use Normal mode, automatic driving-mode switching off, and inherited
response settings at every gap. Startup Params match the 13:03:09 and 13:08:34
settings snapshots on October 4. The latest 16:22:22 snapshot matches 2c5.
These drives are different inputs and configurations, not a controlled A/B test.
Response 4 and a stronger stopping ramp therefore did not eliminate this
recorded symptom; they cannot be isolated as its cause or cure.

## Time and evidence

Time zero is first carState, boot time 54.110569041 s, mapped by clocks to
2026-10-04 13:07:31.517 KST. Segment 1 first carState is t=59.965983 s.
initData reports commit `39157c1cf1f3c8e0b921019fdbe413c08b2c3759`, mici,
carrot-wip and dirty=true. Uncommitted device edits are unknown.

Full cereal schema was used, not the reduced CAN-only schema. Planner/model
publication starts around t=14.5 s; summaries must not backfill that startup
interval with later values. The investigated event is t=48–73 s, with active
longitudinal control, no gas/brake intervention, valid radar/model/plan data,
and xState=lead. Selected lead track32 is continuous from t=50–73 s. Brief
earlier selection variations are visible around t=48–49 s and are not the
explanation for final crawling behind the stationary lead. No traffic-stop
state or navigation speed reduction explains the final approach.

Both qcamera files were sequentially decoded to 1,200 frames. Sampled frames
show approach to a queue at a signal, then crawling behind a white sedan.
Nominal 20 FPS sampling is supporting context, not a precision camera/event
synchronization or physical ranging measurement.

| Time from segment 0 | Ego speed | Lead speed estimate | Logged range | Ego acceleration |
| --- | --- | --- | --- | --- |
| 52 s | 66.4 km/h | 48.2 km/h | 37.2 m | -2.80 m/s² |
| 56 s | 32.6 km/h | 25.3 km/h | 22.9 m | -1.93 m/s² |
| 60 s | 15.8 km/h | 8.5 km/h | 16.8 m | -0.91 m/s² |
| 62 s | 9.0 km/h | approximately stopped | 11.9 m | -0.90 m/s² |
| 64 s | 3.88 km/h | approximately stopped | 8.14 m | -0.45 m/s² |
| 66 s | 2.62 km/h | approximately stopped | 6.22 m | -0.13 m/s² |
| 68 s | 2.10 km/h | approximately stopped | 4.91 m | -0.07 m/s² |

Minimum ego aEgo is -2.897 m/s² at t=52.276 s; minimum planner target is
-3.120 at t=51.796 s. Lead aLeadK minimum is -3.048 at t=50.689 s, but is
radar-derived and transient; do not treat that minimum as exact lead physical
braking or a proof of how hard it had to brake. Peak comparisons alone do not
assess the remaining speed profile.

## Two parts of the tail

The planner already tapers the deceleration as speed falls: target is about
-0.77 at t=62, -0.49 at t=64, -0.25 at t=66 and -0.24 at t=68 s. This
is part of the planned approach rather than an independent acceleration to
recover a suddenly enlarged gap.

Later, actual deceleration is substantially weaker than requested. At t=66 s
the plan's current target speed is 1.80 km/h versus actual 2.62 km/h.
The speed controller requests about -0.47 m/s² while aEgo is -0.125.
At t=68 s the values are 1.05 versus 2.10 km/h, request -0.52 and measured
-0.070. SCC12 sendcan confirms negative raw/value requests of approximately
-0.48 and -0.53, with StopReq=0. Thus the late crawl includes failure to track
the planned slowing, rather than a request to maintain those actual speeds.

At t=69.146 s shouldStop becomes true; long control switches at t=69.151 s,
at actual speed 1.70 km/h and range 4.32 m. CAN then carries StopReq=1 and
the output ramps toward -1.0. Ego drops below 0.1 m/s at t=70.266 s; settled
logged range is about 3.76 m. This means stronger StoppingAccel does not act
as an earlier stop trigger and cannot by itself resolve the preceding crawl.

The target standstill reference is 6 m while logged final range is smaller.
That is evidence of reference-tracking discrepancy; it is not a surveyed
bumper distance. Changing the reference alone is not a demonstrated fix.

The same fixed-plan threshold check used for 2c5 reproduces actual shouldStop
using action time 0.25 s and threshold 0.1 m/s. On the unchanged plans,
0.3 and 0.5 m/s qualify at t=67.247 and 65.596 s instead of 69.146.
This is parameter sensitivity, not a prediction of modified vehicle motion
or a recommendation to change the setting.

Non-CAN-FD SCC14 transmits upper/lower comfort bands near 0.8/0.7 m/s²
during the tail; Hyundai uses speed feedback with Ki=0. Vehicle creep,
actuation, grade and the role of these bands remain unseparated. Do not
infer an ECU fault or prescribe disabling the bands from this evidence.

## Interpretation to retain

### Follow-up: lead acceleration persistence hypothesis

The user suggested that aLeadTau retains the lead's braking too strongly,
causing ego to reduce speed early and then crawl. Isolating the production
`extrapolate_lead` function supports conservative prediction in some portions
of the strong-braking phase. Its formula is
`a(t) = current_aLeadK * exp(-aLeadTau * t² / 2)`; smaller tau retains more
current braking. This is not a direct ego acceleration or jerk command.
Current aLeadK is refreshed on successive updates; tau does not freeze an
old acceleration measurement indefinitely.

Production extrapolation at the logged radar snapshots gives:

| Time | aLeadK | aLeadTau | Predicted lead travel over next 4 s | Subsequently observed lead travel | Same snapshot with tau=1.5 |
| --- | --- | --- | --- | --- | --- |
| 51 s | -2.420 m/s² | 0.00766 | 37.72 m | about 46.24 m | 53.44 m |
| 52 s | -1.569 m/s² | 0.000932 | 38.59 m | about 39.51 m | 49.03 m |
| 55 s | -1.978 m/s² | 0.00000167 | 14.67 m | about 23.70 m | 27.58 m |
| 56.5 s | -0.549 m/s² | approximately zero | 21.94 m | about 16.86 m | 25.60 m |

At 51 and 55 s the prediction places the lead roughly 8.5–9.0 m behind its
subsequently observed position. That gives MPC a more restrictive future
obstacle and plausibly contributes to stronger early slowing. At 52 s the
prediction is already close, and at 56.5 s it overpredicts forward travel.
The bias is not uniformly excessive braking: it depends on subsequent actual
changes in lead acceleration. Restoring tau=1.5 everywhere can make predictions
too optimistic during real sustained braking; it is not a validated remedy.

The observed displacement calculation integrates measured ego speed and adds
the change in radar range. It is an approximate logged reconstruction, not
independent lead ground truth; range bias, road geometry, filtering and service
alignment affect it. Stationary intervals show sub-metre residual drift.
The production MPC discrete integration and horizon interpolation are retained
in predicted values; these are not continuous constant-deceleration distances.
Radar snapshots may differ slightly from the fast liveTracks overlays used at
the exact solver instant. No modified native MPC or closed-loop ego response
was calculated, so the amount of ego braking attributable to tau is unmeasured.

By about t=62.5 s tau has returned to 1.5 and aLeadK is near zero. At t=64,
66 and 68 s, while the final crawl continues, the low-tau persistence is no
longer active. Thus tau can contribute to the earlier profile and its entry
conditions, but cannot alone explain the ongoing weak low-speed tracking and
late StopReq. Both parts must be retained in the diagnosis. The special
jerk-confirmed attack versus ordinary repeated 0.9 decay was not isolated;
do not attribute this result specifically to the September 23 enhancement.

The user is right to distinguish necessary early braking from the later
approach quality. The earlier explanation was incomplete if read as saying
the entire profile was unavoidable because the lead braked hard. This case
warrants examining the transition from ordinary following to final stopping
and low-speed tracking; it does not justify globally weakening lead braking.
No native MPC counterfactual, closed-loop simulation or physical test was run.

## Design review: braking release versus deliberate gap recovery

The user asked whether the delayed response reflects earlier deliberate comfort
behavior. The implementation does intentionally retain extra following headroom
while a gap is opening, and recovers it more slowly for slow leads. Response
levels 0–4 use this policy. Positive acceleration boost separately requires
lead acceleration above +0.1 m/s²: easing from strong to weak negative lead
acceleration does not activate that boost. Ordinary MPC can still release
braking or accelerate without the boost; this is not a prohibition on recovery.

This event does not show ego simply frozen at an excessively low speed while
the lead pulls away. At t=55–58 s, the planner target eases from -2.03 to
-0.77 m/s², and ego is still faster than the lead in the representative
samples. Moreover the displayed reference is approximately the ordinary base
reference at t=56, 64 and 66 s (21.02, 7.29 and 6.48 m). Reconstruction uses
rounded vTargetNow from the detail table and adjacent radar samples, so it is
approximate, but it does not support large extra-TF headroom as the dominant
cause of this specific tail. Do not blame or remove deliberate gap recovery
solely because the symptom resembles it.

A bounded candidate for future evaluation is release of overly persistent
braking prediction after sustained evidence that lead braking has eased,
separate from positive acceleration boost and deliberate gap recovery. Current
tau policy keeps decaying whenever the quiet-reset conditions are not met;
even a change from -2 to -0.6 m/s² need not recover tau. Current aLeadK itself
does update, so some prediction relaxation already occurs. Any additional
release must distinguish noise and brief brake relaxation from a durable change
and retain response to renewed braking. No thresholds or implementation are
selected here. Globally increasing tau, enabling boost for every brake release,
or removing all gap headroom are not supported by this investigation. Final
low-speed actuation and StopReq remain a separate issue.

Private scripts, timeline, CAN summary, plots and video contact sheet:
`.analysis/archive/2026-10-04/niro-2c3/`. Original uploads remain on NAS.
