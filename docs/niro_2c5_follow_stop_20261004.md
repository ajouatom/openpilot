# Niro EV following stops — 2026-10-04

Follow-up clarification: the complaint concerns the prolonged approach after
braking, not necessarily a literal increase in measured range. Necessary early
braking and decreasing range do not establish that the whole speed profile is
unavoidable. See `niro_2c3_stop_tail_20261004.md` for the additional upload and
separate examination of the planned taper, low-speed tracking and stop handoff.

## Finding

The two uploaded segments contain three stops behind the same vehicle. The
lead brakes substantially, and ego initially retains higher speed. Stronger
ego braking is therefore understandable as closing-speed correction; these
recordings do not establish unnecessary overbraking. In particular, actual
range generally decreases through braking, rather than opening into the
reported large gap. The user explicitly raised this alternative during review.

Prolonged final low-speed approach is present. It combines a gradually declining
planned speed, weaker measured low-speed deceleration than the transmitted
request, and a late stopping-state/StopReq transition with VEgoStopping=10.
This does not establish that the behavior cannot be improved with parameters,
or that controller code must be changed. Analysis only: no production code,
vehicle setting, or vehicle state changed.

## Inputs and timing

- KIA_NIRO_EV, route `000002c5--e88ae7f308`, segments 15 and 16.
- Full rlogs and qcamera videos read from the NAS routes share. Private source
  paths, extracted data and reproduction scripts remain in the local archive.
- Full cereal counts: 12,000 carState, 11,999 carControl, 2,400 radarState,
  2,399 longitudinalPlan and 2,399 modelV2. All four inspected input/plan
  services have valid=true throughout. Both videos decode to 1,200 frames.
- First carState is boot time 956.776174768 s; segment 16 starts 59.987786085 s
  later. Times below are relative to segment 15 first carState.
- Clocks map this origin to 2026-10-04 16:36:17.775 KST.
- initData reports commit `39157c1cf1f3c8e0b921019fdbe413c08b2c3759`,
  carrot-wip, mici, **dirty=true**. Exact uncommitted device edits are unknown;
  matching repository source is explanatory evidence, not proof of byte identity.
- Startup Params and the latest available preceding settings snapshot,
  `toggles-20261004-162222.json`, agree on EnableRadarTracks=2,
  TFollowGap1=90, LeadAccelResponse=3, StopDistanceCarrot=600,
  LongActuatorDelay=30, VEgoStopping=10 and StoppingAccel=-50.
  No later settings snapshot was available at analysis time. Runtime tFollow
  is 0.9 s and driving mode is 3 throughout.

## Three stops

| Event | Lead estimated minimum acceleration | Ego measured minimum acceleration | Ego first below 5 km/h | Ego first below 0.1 m/s | Stop request |
| --- | --- | --- | --- | --- | --- |
| 15, first stop | -2.82 m/s² | -3.21 m/s² | 16.809 s | 22.458 s | 21.467 s |
| 15→16, second stop | -3.00 m/s² | -2.98 m/s² | 55.058 s | 61.249 s | 60.280 s |
| 16, third stop | -2.40 m/s² | -2.65 m/s² | 98.949 s | 105.288 s | 104.038 s |

These minima occur at different instants. Lead acceleration is radar-derived,
not the lead vehicle's own accelerometer. Below 0.1 m/s is a reproducible
near-stop criterion, not an exact physical stopping timestamp.

At 12 s ego is 44.5 km/h, lead 33.4 km/h and range 17.6 m. At 52 s these
are 29.1 km/h, 17.7 km/h and 14.6 m. Greater ego deceleration at those times
reduces a real closing speed. Peak command magnitudes also explain measured
braking; this is not evidence of an unrequested brake application.

From first crossing below 5 km/h to near-stop takes 5.65, 6.19 and 6.34 s.
Range at that first crossing is 7.64, 7.43 and 8.98 m; settled logged range
is approximately 4.0, 4.1 and 3.3 m. This supports slow final approach, but
not an increasing gap caused by braking. These are logged radar ranges,
not surveyed bumper clearances.

## What prolongs the tail

At 20 s actual speed is 1.87 km/h against vTargetNow=0.85 km/h.
The planner requests -0.15 m/s², the speed-feedback controller requests
about -0.44 m/s², but aEgo is only -0.046 m/s². At 60 s the corresponding
values are 1.61 versus 0.66 km/h, target -0.18, command -0.44 and measured
-0.037 m/s². At 102 s they are 3.04 versus 1.82 km/h, target -0.27,
command -0.60 and measured -0.025 m/s².

Decoded SCC12 sendcan confirms these negative requests with StopReq=0;
both aReqRaw and aReqValue agree. After the stopping transition StopReq=1,
followed by completion of the stop. All 6,000 sent SCC12 and SCC14 messages
have corresponding message types on Panda TX-echo bus128. This checks the
transmit path, not internal brake-controller execution or exact echo pairing.

Hyundai longitudinal control uses speed error plus acceleration feedforward,
with fixed Kp=1, Ki=0 and Kf=1. It is not an acceleration-error integral
controller. The non-CAN-FD SCC14 path transmits sizable comfort bands
(around 0.76–0.82 upper during the tail). Creep, vehicle response, road grade
and these bands cannot be separated causally using this log alone. Do not
declare a brake defect or assert that comfort-band removal fixes the behavior.

VEgoStopping=10 means **0.1 m/s predicted speed**, not 10 km/h and not a
direct threshold on actual ego speed. `get_accel_from_plan` requires predicted
speed at both action_t=0.35 s and action_t+1 s to be below this threshold.
Only then does the recorded plan request stopping. StoppingAccel=-50 is
-0.5 m/s² and controls the stopping-state ramp; changing it alone does not
advance the planner's shouldStop transition.

Applying alternate thresholds to the unchanged recorded speed plans gives:

| Threshold | First stop request | Second | Third |
| --- | --- | --- | --- |
| 0.1 m/s, recorded | 21.467 s | 60.280 s | 104.038 s |
| 0.3 m/s | 18.221 s | 56.888 s | 102.839 s |
| 0.5 m/s | 17.570 s | 55.779 s | 101.340 s |

The 0.1 calculation reproduces all three recorded transition times. Other
rows are only threshold comparisons on fixed plans: changing braking changes
subsequent vehicle states and plans. They do not predict stopping distance,
comfort or prove a suitable setting. They do demonstrate material parameter
sensitivity, so a parameter-independent defect is not established.

## Alternatives and limits

LeadOne remains valid track41 in all 2,400 radar messages; LeadTwo is absent.
xState stays lead, never e2eStop. Experimental mode and forceDecel stay false;
no gas or brake pedal intervention occurs, and set speed remains 70 km/h.
No speed-camera constraint appears. Minimum navigation curve target is above
the low-speed stop trajectory. DM warning events occur elsewhere but do not
request forceDecel. Lead preview and gap preference can shape trajectories;
their isolated contribution to braking comfort was not replayed through native
MPC and must not be inferred from source labels alone.

Sampled video frames show the same lead braking in urban traffic, consistent
with radar continuity. No physical test or modified closed-loop simulation was
performed. Review low-speed request tracking and stop handoff separately from
the justified stronger braking that removes closing speed.

Private evidence: `.analysis/archive/2026-10-04/niro-follow/`.
