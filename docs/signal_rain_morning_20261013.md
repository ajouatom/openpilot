# Rainy morning signal investigation and preparation receipt-time correction

## Evidence and installed version

Read-only collection retrieved 108 source files / 1,173,893,906 bytes, all checked
against device SHA256. This includes inventory qlogs and 23 completed driving
segments with detailed logs and road videos: 10a7/10a8 on October 12 evening and
10a9 at 06:47–06:59 on October 13. Two locked inventory segments were excluded.
Vehicle HEAD was carrot-wip `2ec25da3c1`, SignalAssistMode 2, old moving assist OFF.
10a7 recorded nav `d77f11ce35`; 10a8/10a9 recorded wip `2ec25da3c1`. The single
central red-to-green veto-release correction was therefore installed for these
drives. No vehicle files, settings, models or running processes were changed here.

## Three separate control paths

1. `CarrotPlanner` contains state 6, `e2eStopPrepare`. Eligible auxiliary red sets
   this state. `LongitudinalPlanner.apply_signal_stop_prepare` additionally caps
   final acceleration toward -0.5 m/s² at 0.25 m/s³. It does not merely relabel
   xState, set model green/red, supply a stop-line distance, or force shouldStop.
   Both preparation eligibility and final output explicitly reject either radar
   lead. Gas, brake, turns and existing model stopping also reject preparation.
2. The separate stopped-red guard can latch below 0.3 m/s. It overrides the
   planner to e2eStopped, zero cruise/stop distance and final shouldStop/velocity
   veto. By the earlier policy it persists even when a lead appears or moves.
   This conflicts with the user's expectation of unconditional lead following.
   Its `departure` input currently includes mere lead presence; the warning can
   therefore sound while coming to rest behind a stationary car.
3. Original model/lead stopping remains independent. A model stop after auxiliary
   preparation is not cancelled just because auxiliary observations disappear.

Changing the moving preparation's lead gate would not fix the separate stopped
veto. This patch changes neither policy and does not turn lead presence into
permission to ignore a held red.

## What actually happened

All three collected drives have **zero logged active preparation and zero
xState=6 samples**. Today's two audioRefuse events at 06:55:35.573 and
06:55:41.823 are stopped-red warnings, not trafficSignChanged preparation sounds.
The guard was cleared by disengagement between those stops, then reacquired.
LeadOne was present, approximately 8.6 m and 3.7 m respectively, with ego speed
about 0.9 km/h. Today's deceleration was not caused by the -0.5 preparation cap.
The guard did intervene near rest and retained the stop afterward.

Morning: 958 observer samples contain confirmed current red; 357 are while
moving above 5 km/h, and 340 of those have a radar lead. The remaining 17 have
driver action, turning or existing stopping exclusions. The observer-sample
screen finds zero eligible moving-red contexts; these counts are not accuracy.

Evening: 10a7/10a8 contain 316/406 confirmed-red observer samples, including
121/105 while moving. There was recognition without preparation, not a complete
absence of red detection. Among preliminarily eligible observations, small red
identity loss, missing a second supported lamp, road geometry and competing
green hypotheses prevent many entries. Some 10a8 red candidates are vehicle
lights in scenes with actual overhead green; indiscriminately relaxing these
gates would be inappropriate. Selected video sheets were visually inspected.

## Reproducible runtime bug

`SignalPrepareRuntime.read` obtained its validation time from the base transport
reader **before** calling `extra.update(0)`. `SubMaster.update` calls
`update_msgs(time.monotonic(), ...)` and timestamps newly received camera,
calibration and device data during that call. Thus `now - recv_time` was negative
for valid newly received data. The nonnegative-age check rejected it and the
preparation gate cleared confirmation as `ineligible`.

The evening trace around 21:51:01–04 repeatedly alternates `confirming` and
`ineligible` while real red tracks persist. A recorded-input replay which bypasses
the runtime reader can enter preparation for 13 planner updates at
21:51:03.532–04.130; the live log never enters. This discrepancy is consistent
with the proven reader bug, but exact live SubMaster receipt times were not logged,
so we cannot attribute every rejected live update to it.

The fix resamples monotonic time **after** receiving extra inputs on the live
clock path and rechecks main input ages at that same time. Pose/camera checks use
this time too. Explicit replay-time inputs keep their supplied clock. No future
timestamp tolerance or freshness limit is widened. Before the fix, the new
receive-order regression fails; after it, repeated newly received observations
can confirm, while genuinely future/stale inputs and inputs expiring during a
receive remain rejected. Existing lead/cancellation tests remain unchanged.

The ideal-arrival replay's brief preparation begins only 0.647 s before logged
native model stopping. It is not evidence of sustained -0.5 braking, a full stop,
or a post-fix vehicle response. The replay uses sampled messages and existing
planner substitutions; it does not validate native MPC or the reader itself.

## Morning green hold: incorrect lamp location inside the tracked box

In 10a9 segment 9, both overhead signals visibly change to green while the tracked
boxes continue to report unknown. The guard does not release before manual
disengagement/gas around 06:56:49.96, even as the lead starts moving and the original
model velocity rises. This is not a missing single-transition release feature.

Exact encoded frame 341 (about 06:56:48), resized to the observer's 1344×760:

| Tracked box | Existing raw/verified result | Right edge shortened by 4 px |
|---|---|---|
| [622,309,654,325] | unknown, green score 0, verification false | green, score 0.547, verification true |
| [575,309,607,325] | raw green, verification false | green, score 0.631, verification true |

The fixed right-lamp fraction and the estimated wide box place the inspection
region beyond the actual light. A separate enlarged original-pixel crop confirms
the mismatch. This is a diagnostic crop probe, **not a shipped four-pixel fix**:
encoded pixels differ from live NV12, and shrinking every box could misclassify
other housings/arrow layouts. The recognizer needs housing/layout-aware lamp
placement and red/green transition regression, rather than a shorter confirmation
delay or treating unknown as green. This perception correction is not implemented
by the receipt-time patch.

## Cadence and GPU

Morning CPU observer: 6,294 observations, about 8.87–10.79 Hz by segment.
Median capture interval is approximately 100 ms. Median input age is 48.53 ms,
work wall time 52.89 ms, process CPU time 41.85 ms, total result age 102.86 ms
(95th percentile 132.58 ms). The worker sleeps for at least its process CPU time
to target half of one CPU; its advertised 20 Hz is a ceiling, not measured speed.
Green release also needs at least 700 ms of fresh matching evidence. Faster
execution alone cannot turn the observed unknown evidence into green.

A desktop profile of 69 selected stored frames spends 0.456/0.495 seconds in
candidate detection, especially housing/day searches and repeated image color
conversion. This uses fresh trackers and desktop hardware, so it only prioritizes
optimization work; it does not establish vehicle speedup or steady tracking cost.
Reuse image transforms and do less broad search between verified tracking updates
before increasing CPU duty. Any cadence change needs model/DM timing measurement.

The earlier parked GPU A/B/A experiment measured DM about 10.6–10.8 Hz with the
QCOM helper and about 20 Hz with CPU; see `signal_dm_flicker_20261011.md`. Mode 2
currently explicitly chooses the CPU tracker. This demonstrates contention for
that GPU workload, not that every GPU implementation is impossible. Today's DM
inference median is about 30 ms; 591 cameraUnavailable state samples and long
outliers also exist, so this investigation does not claim an entirely healthy DM
run or blame those outliers on an unused QCOM helper. DM protection is retained.

## Validation and scope

537 focused desktop tests pass with the existing Windows Params storage stub,
including five new receipt/freshness cases. Publish the same runtime correction,
tests and this report to carrot-nav and carrot-wip using single-parent commits.
The ONNX, green detector, stopped-red lead policy, CPU duty and GPU selection are
unchanged. Vehicle installation and road validation have not been performed.

Private source manifests, decoded data, replay, performance results and visual
review are under `.analysis/scratch/2026-10-13-signal-morning/`. Keep them private.
