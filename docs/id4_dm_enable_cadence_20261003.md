# ID.4 DM re-enable communication fault — 2026-10-03

## Finding

`000001c5--b6895edc96--13` records DM disabled at segment start, its live
re-enablement, excessive `driverMonitoringState` publication frequency, and two
communication-fault disengagements. This is not evidence of a network-speed
problem. The offending service is DM state; driving-model output, camera
streams and CAN remain healthy in the inspected segment.

The producer combines a full 50 ms camera-message poll with an independently
accumulating 20 Hz Ratekeeper. While the DM model is stopped, every iteration
takes the full poll timeout plus processing. Ratekeeper falls progressively
behind and does not discard missed deadlines. Once model output resumes,
timeout iterations and early message-arrival iterations can both publish,
while Ratekeeper skips sleep to catch up. This mechanism reproduces excessive
publication using the recorded model-arrival sequence and synthetic prehistory.
The actual accumulated timer deficit is not logged, so its exact size and the
duration of preceding disabled operation are not measured here.

The initial investigation was analysis only. The user subsequently requested
correction and deployment; the implemented correction and validation follow
below. No health thresholds or DM policy criteria are changed.

## Provenance

- Full rlog, SHA-256
  `b89c4921c3b14c7182196c3906ba5e9204c2fdb8969ceb75e4f54632e4bfa675`.
- C4/mici, `VOLKSWAGEN_ID4_MK1`, openpilot longitudinal control.
- Recorded repository `helico717/openpilot`, branch
  `carrot-wip-model_selector-ha`, clean commit
  `879d57a3c3434453ba34d3a589d7456369e7b01e`.
- That commit changes HA trip-battery accounting, not DM scheduling. The DM
  dispatcher, Ratekeeper and messaging frequency checker are byte-identical
  to carrot-wip `45e10ac94d` in the relevant files.
- Full cereal decoding; all times below are relative to first carState,
  monotonic 1095.469415969 s. Only this 60-second segment was available for
  this route on the NAS.

## Recorded timeline

| Segment time | Observation |
|---|---|
| 0.00 s | OP already enabled; DM reports disabled. |
| 10.157 s | Manager starts dmonitoringmodeld. |
| 10.830 s | DM reports enabled, temporarily using interaction monitoring. |
| 12.112 s | DM models loaded. |
| 13.673 s | First driverStateV2; first inference takes about 1.562 s. |
| 15.895 s | Camera monitoring becomes available after health qualification. |
| 20.780 s | First invalid driverMonitoringState packet. |
| 20.794 s | commIssue soft-disable begins. |
| 21.771 s | commIssueAvgFreq alert first appears. |
| 23.792 s | OP becomes disabled after the three-second soft-disable interval. |
| 33.800 s | Driver re-enables OP. |
| 44.812 s | Communication faults recur intermittently. |
| 49.252 s | Another continuous soft-disable interval begins. |
| 52.251 s | OP becomes disabled again. |

The user's reported sequence is broadly consistent with re-enabling DM and
subsequent disengagement, but OP was already enabled at this segment's start;
the log does not show a new OP enable between the DM toggle and first fault.

## Communication evidence

Before DM model startup, state output is 18–19 messages per one-second bin.
Afterward it reaches 25–29; driverStateV2 remains around 20. DM publishes 1,407
state packets in this segment, including 472 invalid packets. At 21.776 s,
selfdrived records DM state as alive, only 1.988 ms old, but invalid, with
average/recent frequencies 24.073/24.456 Hz against the unchanged 16–24 Hz band.
Other checked model/planning services in that diagnostic remain healthy.
`alertDebug` also appears in the diagnostic but is explicitly ignored for
alive, validity and frequency; it is not the disengagement cause.

DM constructs its packet validity from carState/selfdriveState checks. Their
conflated inputs are expected to be sampled by a 20 Hz loop; an overspeed loop
can fail these input frequency checks even when their 100 Hz publishers are
healthy. Reconstructing those two FrequencyTrackers using logged DM output
times minus an assumed 3 ms receive-to-publish delay agrees with 1,024 of 1,047
validity flags after 18 s (97.8%). Assumed delays of 0/1 ms agree on 950/985.
This supports the mechanism but is not an exact replay of unlogged internal
receive timestamps or proof of which individual input check first failed.

All 5,999 carState messages are valid with canValid true, no CAN timeout and
no temporary/permanent steering fault. All 1,200 livePose messages have
inputsOK, sensorsOK and posenetOK true. Road/wide/driver camera frame IDs are
consecutive (1,200 messages each), as are modelV2 frame IDs (1,201 messages).
driverStateV2 skips 31 old camera frames after its long first inference, then
has consecutive frame IDs. Its later maximum publication gap is 86.589 ms;
DM-state maximum gap is 57.384 ms. The evidence is excessive state frequency,
not a stopped DM heartbeat. Manager samples retain stable expected process
PIDs, apart from the intended DM model start; no subsequent restart is seen.

## Initial reproduction

The private experiment extracts the actual Ratekeeper and FrequencyTracker
classes from repository source. A synthetic 780-second disabled prehistory,
3 ms processing per iteration, full 50 ms poll timeout, and the recorded DM
model-arrival times produce 44.873 s accumulated lag at first model output.
It produces 26–29 Hz in seconds 20–26 and 24–29 Hz in seconds 50–56, matching
the type of failure. These prehistory/processing assumptions are illustrative,
not recovered device measurements. Starting the simulation at segment time
zero accumulates only 0.723 s and does not reproduce the sustained fault,
which shows why startup-only tests can miss this transition.

A comparison using a single 20 Hz schedule with nonblocking message reads
eliminates that timeout debt and keeps these synthetic windows at 20 Hz.
This comparison was exploratory. The final correction retains camera polling
and its existing frequency checks, and instead bounds the timer's outstanding
deadline, as described below. It does not introduce nonblocking device polling.

This initial experiment is not a native IPC test, CPU/GPU diagnosis or vehicle
validation. Private scripts, logs and results are indexed under
`.analysis/archive/2026-10-03/id4-dm/`; raw vehicle data are not committed.

## Implemented correction

Device DM uses a dedicated `DmRatekeeper` that retains the 50 ms cadence but
rebases the next deadline on actual completion when an iteration or sleep is
late. Missed deadlines are discarded instead of accumulating indefinitely.
The existing driverStateV2 poll, its 50 ms timeout, all SubMaster frequency
limits and the output validity calculation remain unchanged. A bounded phase
adjustment can still produce an occasional short publication interval; there
is no accumulated backlog of iterations to repay after re-enabling DM.

Replay retains its original Ratekeeper, modelV2 polling and route-time clock.
The common Ratekeeper used by other processes is unchanged. DM policies,
camera fallback/recovery, disabled heartbeat, lockout and non-conflated
physical input reads are unchanged. Settings retain their intended behavior;
this corrects the communication failure when that behavior is exercised.

Validation:

- 283 focused policy/dispatcher/cadence/parking/touch/parser tests pass on
  Windows with native IPC/Params/hardware adapters and real cereal messages.
- New end-to-end dispatcher tests keep the production DM timer and actual
  FrequencyTracker class. They simulate 780 s disabled, model startup,
  alternating late/early inference, camera outage and recovery in both modes.
  Replacing only the timer with the previous Ratekeeper fails both tests;
  the corrected timer passes. Native transport is simulated, not validated.
- Twelve timer tests cover 0/60/780/3,600 s absence, sleep overshoot,
  0.1/1/30 s stalls and genuine sustained overload. Overload remains below
  the existing health limit rather than being hidden by valid flags.
- With recorded ID.4 model arrivals, the same synthetic 780 s prehistory and
  3 ms iteration work, the old/new timer produces 1,443/1,186 packets over the
  segment and 787/0 output FrequencyTracker failures. This is a scheduling
  experiment, not an exact replay of device scheduling or vehicle response.
- Focused Ruff checks pass. A standalone Linux cadence CI job and the full
  build's DM regression coverage are included.

The correction is intended for `ajouatom/openpilot` carrot-wip. The reported
`helico717/openpilot` fork is separately maintained and the current account has
no push permission there; upstream publication does not update that fork or
install software on the reported device. On-device re-enable and driving
validation remain outstanding.

Correction scripts and comparison results are retained separately under
`.analysis/archive/2026-10-03/dm-cadence-fix/`.
