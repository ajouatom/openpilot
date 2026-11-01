# Manual indicator response and one-touch feedback — 2026-11-01

The latest drive ran 441d71e330. Eleven closed, SHA256-matched full-schema
rlogs (147,479,258 bytes) provided 12,755 model frames. Private route identifiers,
raw CAN, traces and replay artifacts remain outside committed documentation.

## Findings

Twenty manual indication requests were observed. Fifteen had matching CAN lamp
feedback within two seconds: median 0.206s from request, range 0.147–0.254s.
This is a feedback measurement, not a camera-confirmed lamp observation.
The median nearby initial steering-torque-to-request interval was 1.547s.
The fixed 0.9m boundary threshold contributed to late requests.

Our transmitted one-touch also appeared in carState.blinkerLever. Both the
model observation and the comma source treated that raw field as driver
cancellation, including while they owned the request. Stock neutral frames
interleaved our tap frames, sometimes incrementing the stalk counter twice.
A completed-send ACK can arrive after the first observed lamp-on interval;
previous source logic forgot that earlier evidence.

Not every missed indication has this cause. Low speed, left yellow markings,
paired-boundary inconsistency and return-before-crossing also blocked/ended
requests. Two accepted sends without lamp feedback remain unexplained; this
change does not establish that they will illuminate.

## Changes

- Manual boundary admission projects 0.3s using measured, smoothed approach
  speed. The centre-to-boundary threshold is min(1.2m, 0.9m + speed*0.3s).
  Real approach >=0.15m, paired-boundary consistency, intent, confidence,
  30km/h minimum, left white-line and right edge clearance gates remain.
- The comma source publishes a small local tap context only after successful
  socket submission. It binds request ID/time/direction and initial stalk
  counters, with fresh receiver state. It uses the existing shared-memory
  hold file, not a new USB message or CAN signal.
- Model observation and source admission recognize only that matching short
  echo: context age <150ms, original send window <600ms, tap duration <250ms,
  same-side counter increments bounded by the existing three/four tap frames,
  no opposite counter/lamp and no latched lever. No new requests use this
  exception. Receiver driver-override and unchanged-physical-stalk checks stay.
- Same-direction physical input during that short interval is not uniquely
  identifiable from aggregate carState fields. This is a bounded echo rule,
  not proof that the source can always identify the driver. Receiver override
  remains independent; outside the window raw input cancels normally.
- An observation may remember early matching lamp evidence for <750ms. The
  exact request's completed-send ACK is still required before physical hold;
  stale/invalid/driver-override/opposite feedback clears that memory.
- ATC/TBT/BT policy, receiver frames, Panda safety, automatic-steering isolation
  and model/runtime artifacts are unchanged. Manual/model lamp-only origins
  still cannot request automatic steering.

## Verification

Controlled replay preserves recorded vehicle inputs and ownership journals.
Before/after both produce twenty manual requests, with no additional candidates
in this replay. Fourteen occur earlier: median advance 0.298s, maximum 0.760s;
six do not advance because other gates still decide admission. Recorded
ownership is held fixed, so this is not a closed-loop prediction of a new drive.

A separate cancellation replay supplies the new local transmission context at
the recorded request, while preserving recorded CAN counters and geometry.
Thirteen old `cancelled` endings instead reach geometry/return termination;
one ends in the same sample due to actual return. Example original segment
4 at 21.543s: cancellation at21.795 becomes geometry termination at22.592.
The local context was not logged by the old code: this is an offline regression
scenario, not evidence that receiver ACKs or real lamps now work in the vehicle.

Desktop indicator/HUD suites: 700 passed, 2 platform skips. The 21 new tests
include actual source/model roundtrips, interleaved neutral frames, counter
wrap, delayed/missing ACK, driver override, stale/mismatched contexts, pulse
limits and prediction bounds. The prior expiry fixture releases torque earlier
so its intent actually expires before the new predictive threshold.

## Delivery

Normal carrot-wip Git Pull and comma restart. This is comma-side code; no new
Jetson signed package, engine build, model selection or SD/USB patch is needed.
No vehicle writes, live CAN actuation or reboot were performed in this task.
Physical lamp/road acceptance remains pending.

Publication: source 6054829dc83826e54b4698b07c9874d313910401 on carrot-wip.
Signal CI41273870185 passed 1277 tests/40 skips and362 subtests. Jetson
CI41273870206 passed2214/56skips plus232/62/94/119/11/11 supporting tests;
Windows installer passed. eGPU41273870261 and user-docs41273870219 passed.
Wiki manual7589726 and automatic sync c8a0cf2 published; all nine affected
pages have current semantic review hashes and valid markup (sync41273901339).
The separate general release-build workflow was still running at this record;
its relevant web, lifecycle, DM cadence, lane-mode and eGPU startup jobs passed.
