# Ioniq 5 PE 5d7: reduced steering authority and missed handover

Analysis of `000005d7--d77ade4413--0`, clean `abe1a232`, prompted by the
report of drifting between lanes around video second 45. This investigation
changes no vehicle-control code or settings.

## Findings

Two distinct software weaknesses are present:

1. Before the first model output, AlwaysLateral enables lateral control and
   sends nearly zero-degree steering targets. Model readiness is missing from
   that activation gate. This is independent of the experimental handover modes.
2. Around 41 seconds, mode 3 consumes its rapid-release arming evidence on an
   early capture, aborts on a short raw-force rebound, and cannot retry on the
   subsequent sustained release. Legacy recovery then takes over with its
   accumulated three-second ramp. Steering error grows while authority is low.

The latter matches the reported handover difficulty; the former also matters
when interpreting the unusually frequent overrides and low authority earlier
in the recording. Neither driver intent nor the entire lateral trajectory can
be reconstructed causally from the torque signal alone.

## Evidence and timing

The full cereal schema decoded the NAS rlog. Road frames were selected using
`qRoadEncodeIdx.segmentId` and capture `timestampSof`. Times below are relative
to the first carState, monotonic 56.142839925 seconds. Publication, CAN batch and
camera-capture times differ by milliseconds; do not interpret them as a single
hardware timestamp.

Mode 3 is confirmed by both initData and live controller diagnostics. The car
uses Hyundai angle control with `AlwaysLateral=1`. LFA_ALT's
`LKAS_ANGLE_MAX_TORQUE` is an authority ceiling in CAN units, not a measurement
of physical steering torque. A ceiling of 250 does not mean that 250 units of
actual steering force are continuously applied on a straight road.

### Startup activation before model readiness

- Model loading starts at -0.432 s and finishes at 24.381 s: 24.8 seconds.
- Camera-pair-unavailable/out-of-sync messages follow until about 26.75 s.
- First recorded modelV2 is at 29.129815 s; livePose and lateralPlan first
  appear at 29.168428 and 29.228083 s. Modeld explicitly reports its first eGPU
  output/startup completion at 29.184 s.
- Nevertheless, carControl is laterally active from approximately 7.31 s,
  while selfdriveState is disabled. Targets settle to approximately zero.
- Panda bus-128 TX echoes show active LFA_ALT from 7.347424 s, with nonzero
  ceilings including 250 before the first model output. These echoes verify
  transmission, not the vehicle's physical acceptance or delivered motor torque.
- At 7.3–29 s, 2,169 laterally active samples include only 251 at full ceiling
  (11.6%) and 1,384 at minimum ceiling (63.8%). Repeated driver-force detections
  accompany this period; their physical cause is not established.

`controlsd.lateral_control_allowed()` combines selfdrive active OR AlwaysLateral
with gear/latEnabled, steering faults and speed checks. It receives no model or
planner readiness/freshness input. The ordinary non-lane-line path then reads
the default model action's desired curvature. The handover helper rejects
missing model evidence, but in its waiting state returns the legacy ceiling;
it does not disable lateral control. Consequently its validity check does not
repair the startup gate.

This is not evidence that the model was running and commanding a bad path
before 29 s. It had not published its first output. The precise camera-pair
startup delay and all upstream eGPU startup costs are outside this analysis.

### Handover near the reported incident

| Approximate time | Recorded behavior |
| --- | --- |
| 41.205 s diagnostic | `capture`; ceiling 26.7, legacy 25 |
| 41.230 s diagnostic | `blocked`; ceiling returns to 25 |
| 41.99 s | Valid touch status changes to no contact; ceiling about 25 |
| 42.00 s video | 124.5 km/h; target 4.4°, actual 1.4°, ceiling about 26 |
| 42.622 s | Target-minus-actual error reaches 4.14°; ceiling 73 |
| 43.00 s video | 124.2 km/h; target 6.3°, actual 2.4°, ceiling about 102 |
| 43.76–43.79 s | SteeringPressed and touch return; authority drops again |
| 44.00 s video | Actual 9.2° versus target 6.4°, strong positive column force; ceiling 25 |
| 44.425–44.865 s diagnostics | Brief convergence offer, followed by another force-based block |
| 46.016 / 46.051 s diagnostics | New capture, then rapid recovery |
| 46.186 s | Valid touch status becomes no contact again |
| 46.533 s carOutput | Ceiling reaches 250 and remains there through segment end |

Video frames show substantial lateral repositioning relative to the lane
markings over 42–46 s. This supports taking the reported difficulty seriously;
it does not provide calibrated lane-boundary crossing distances or prove the
driver's intended lane. Both blinkers remain off in 35–50 s.

At 29.5–46.53 s, only 178/1,703 samples (10.5%) have full authority; 631/1,703
(37.1%) are at minimum. At 46.54–60 s, all 1,338 available samples have ceiling
250 and no steeringPressed. Low ceilings are therefore not an unavoidable
property of this vehicle or of nominally straight-road steering.

## Why the first rapid recovery fails

Replay of the unchanged helper and unchanged legacy controller reconstructs:

1. At carState time 41.199874 s, driver signal 169 gives normalized raw effort
   0.676. Capture starts and clears `armed_at`.
2. At 41.217054 s, the driver signal briefly rises to 262 (1.048 normalized).
   SteeringPressed is still false. Capture's raw-force veto is
   `max(0.8, capture_effort + 0.15) = 0.826`, equivalent to 206.5 signal units.
   The single-frame raw value exceeds it and cancels capture. The live log's
   41.205→41.230 s state transition independently confirms the event.
3. Fresh arming requires at least 150 ms of continuous pressed/high-force
   evidence. After this abort and before the sustained low-force period, replay
   finds at most about 10 ms of accumulated strong evidence. Short subsequent
   pressed pulses cannot rearm it.
4. Once low force persists, blocked→returning→waiting occurs at about
   41.95 s. Because experimental and legacy ceilings are both 25, handback
   completes immediately. It resumes the independent legacy ramp, not rapid
   recovery. The repeated-override count is already 3, setting a three-second
   full-scale ramp, approximately 75 ceiling units per second.

The resulting weakness is loss of the release opportunity after an aborted
early capture. It is not the retired two-degree entry gate: this implementation
has no angle-error gate for starting rapid recovery. Error scales the rise rate
after confirmation. The later release at 46 s succeeds on the same code.

No conclusion that the 262-unit sample was noise, road reaction, or deliberate
driver opposition is justified. Current logic treats all three alike. Raising
the veto threshold alone would trade missed recovery for weaker driver yielding.

## Touch and pressed are different measurements

`HyundaiSteeringTouch` accepts original ECAN STEER_TOUCH_2AF only with matching
layout, checksum, counter progression and freshness (250 ms). Valid
`rawStatus >= 1` means touched; zero means no contact. `rawTouch1` and
`rawTouch2` are exposed for diagnostics, not thresholded independently.

Touch is valid throughout 35–50 s:

- Contact until 41.987206 s.
- No contact until 43.785849 s.
- Contact until 46.186495 s, then no contact.

The slight mismatch between force/pressed and touch transition times is not by
itself a sensor fault. Contact reports do not establish steering intention.
SteeringPressed uses absolute column torque above 250, confirmed for more than
five consecutive carState updates. Touch is not an input to this handover helper.

## Validation and limits

Over 35–50 s, 1,499 replayed carState samples match recorded ceilings within
0.133 CAN units maximum (99th percentile 0.024; none exceeds 1). Small differences
come from reconstructing controller timing with logged publication times. This
validates the failure-state reconstruction, not a modified closed-loop response.

In this same window, modelV2, livePose, lateralPlan, carControl and carOutput
have no invalid publications. Maximum logged model age at carState is 59.1 ms;
maximum yStd[10] is 0.1092. Original MDPS fault bits are zero in all 1,500
examined frames. Target versus limited command differs by at most 0.080°.
The previous 5d3 incident's 175° command limit is not implicated here.

Possible next work should separate the two issues: gate AlwaysLateral on usable
control inputs, and review bounded retry/confirmation after aborted release
capture while preserving prompt yielding to renewed driver effort. Touch can
corroborate a release but is not driver consent or an instruction to restore
full authority. No revised thresholds, recovery policy or vehicle fix is
validated or installed by this investigation.

Reproduction scripts, numeric extracts and synchronized frame/plot evidence
are retained privately under
`.analysis/archive/2026-10-02/ioniq5-5d7-steering/`. Raw route/video remain on
the original NAS; neither raw captures nor settings snapshots are committed.
