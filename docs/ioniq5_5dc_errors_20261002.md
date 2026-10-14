# Ioniq 5 PE 5dc segment 2 error investigation

Initial analysis on 2026-10-02; subsequent user-authorized revert is recorded
below. No vehicle settings or running-device changes were made.

## Evidence

- Vehicle: HYUNDAI_IONIQ_5_PE, device 7f419ed56030e135, C4/mici.
- NAS route: `000005dc--5c3dcd82e4--2/rlog.zst`, 15,766,943 bytes.
- SHA256: `a5f56dfb0d925cff12f12db633eb29b9dadd76b0c6d7d759cd1eb3c08b4ff5f6`.
- Full cereal schema confirms clean `a720c53aafb86b68b00a94e8fcb4fdb8eff5311c`, branch carrot-wip.
- Time zero is first carState, monotonic 179.611267444 s. Coverage is about
  60 seconds; these are log-relative times, not an exact video alignment.

## Main error: repeated locationd temporary error

The recorded UI repeatedly reports `locationdTemporaryError/softDisable`
with `TAKE CONTROL IMMEDIATELY` and `warningSoft`, plus no-entry refusals.
This is present at the segment boundary. There are 2,739 soft-disable-alert
samples and 363 no-entry-alert samples at approximately 100 Hz. Alert hold
time makes these counts different from underlying fault duration.

`livePose.inputsOK` is false on 83 of 1,156 messages, in 42 distinct runs,
between +0.015 and +41.999 s. `sensorsOK` and `posenetOK` remain true on all
messages. The error concerns input validity, not evidence of a GPS fix loss,
IMU hardware failure, or locationd process crash.

There are 44 modeld `camera dropped 1 frames; advancing model history`
errors and 44 frame-ID steps of two among 1,157 modelV2 messages.
cameraOdometry has 45 invalid messages, including one at the start boundary
whose preceding model iteration is outside this segment.

The source at the recorded revision explains the propagation:
`fill_pose_msg` marks cameraOdometry invalid when model input frames are
skipped; locationd includes message validity in inputsOK. Calibrationd also
propagates its input checks into liveCalibration validity. Eight invalid
liveCalibration messages and 39 invalid liveParameters/driverAssistance
messages explain additional `commIssue` events. All 233 calibration status
values still say calibrated; this is not a calibration reset.

commIssue diagnostics occur approximately +5.27 to +19.58 s. `alertDebug`
also appears in their diagnostic lists, but its alive/valid/frequency checks
are explicitly ignored; it is not the blocking service.

All ten enabled-to-disabled transitions coincide with `pedalPressed`
and recorded brakePressed=true. The system repeatedly enters softDisabling,
but this segment does not demonstrate expiration of that timer as the
reason for disengagement.

## Camera processing delay and CPU contention evidence

All three cameraState streams contain 1,201 messages and uninterrupted
frame-ID increments of one. Their recorded SOF intervals average 50.0 ms,
with maxima 57.90/57.95/57.89 ms for road/wide/driver. Thus the model input
skips are not missing frames in the recorded cameraState sequence.

| Stream | Mean publication age from SOF | Maximum age |
| --- | ---: | ---: |
| Road | 90.64 ms | 247.14 ms |
| Wide | 91.35 ms | 276.62 ms |
| Driver | 76.63 ms | 243.54 ms |

Model runtime diagnostics report camera age at inference mean 91.84 ms,
maximum 266.69 ms, while inference itself averages 38.37 ms and peaks at
40.91 ms. This localizes a substantial delay before inference.
Recorded SOF timestamps are kernel-derived, not an independent physical
sensor clock; these data do not prove a sensor hardware cause or its absence.

Whole-core6 usage averages 93.99%, maximum 98%. Process CPU deltas over the
segment are UI 39.00%, selfdrived 25.38%, controlsd 19.60%, camerad 7.98% of
one core. Their sampled main-thread processor is core6; this does not measure
every worker's affinity or establish which process delayed a particular frame.

| Window | Core6 mean | Wide publication age mean | Model skipped frames |
| --- | ---: | ---: | ---: |
| 0-20 s | 95.66% | 112.15 ms | 29 |
| 20-40 s | 95.09% | 91.83 ms | 13 |
| 40-60 s | 91.23% | 70.11 ms | 2 |

CPU contention is a strong investigation candidate, with UI/control/camera
sharing core6. This is correlation, not a scheduler trace or controlled
placement experiment. Do not label UI, the native radar change, or the
driving model as a proven root cause. card/radard both report the Cython
backend; no matching Python/native A/B test is present here.

## Separate Panda SPI error burst

Between +48.903 and +50.461 s, 25 `spi_failure_diag` records show HACK/DACK
NACK retries. All recorded final results are nonnegative after 2-3 attempts.
Panda's SPI checksum counter increases from 2 to 38, a delta of 36, with
firmware header/data checksum diagnostics. Firmware diagnostic print time
need not equal the original fault instant.

This is a host-to-Panda SPI transport problem, distinct from decoded vehicle
CAN faults. All 5,995 carState messages have canValid=true, canTimeout=false,
and no temporary/permanent steering fault. All radar errors remain false.
Across 523 pandaStates, there is no fault, heartbeat loss, RX/TX buffer
overflow, safety RX-check invalidity or safety TX block. Existing CAN RX-loss
and core-reset counters do not increase between the endpoints.

The SPI burst occurs after the last locationd input failure and does not
explain the earlier recurring warning. The transport's underlying cause is
unresolved; successful retries do not prove that every wire-level deadline
was unaffected.

## Other checks and retained evidence

All 105 managerState snapshots have unchanged expected process records/PIDs
and no expected process stopped. Thermal status is green throughout;
memory usage is 58-59%. No CAN/steering/radar fault is shown in this segment.

Next useful diagnosis is a controlled parked reproduction measuring camera
thread scheduling/runqueue delay on core6, with UI workload and camera age
recorded together. Investigate the SPI burst separately. Do not weaken pose
validity checks to conceal these alerts.

Private decoding script, log summaries and reduced evidence are retained in
`.analysis/archive/2026-10-02/ioniq5-5dc-errors/`; the raw rlog remains on NAS.

## Follow-up: revert core tune (#531)

The user requested immediate reversion if the core-tune commit was found
problematic. Commit `68657e2cbec4d88ca0c4bc87679d7c0ee27b6ac5` changes exactly
one line: shared `DISPLAY_NICE` from 19 to -10. It does not change affinity.
This promotes display workers over normal-priority camera work instead of
keeping the established low-priority display policy. It also affects the
cluster and its encoder through the shared scheduler.

All 31 procLog snapshots confirm UI nice=-10 and camerad nice=0, both sampled
on core6. controlsd/selfdrived retain real-time priority. This establishes that
the changed UI priority actually applied in the incident, strengthening the
CPU-contention explanation. It does not by itself measure each scheduling
delay or prove the sole cause of all camera input skips.

The exact core-tune change was reverted, restoring DISPLAY_NICE=19. Existing
display-scheduler tests reproduce 10 failures and 1 pass before the revert,
then all 11 pass after it. These cover UI/cluster workers, encoder children,
onroad/offroad transitions, CPU hotplug and C3 shared-core placement. Tests
run with their directory as pytest confcutdir because the Windows workstation
lacks the unrelated compiled Params extension imported by root conftest.

The native CPU kernels, core affinity, camera/control/model priorities and
later startup-recovery commit are unchanged. No pose validity threshold is
relaxed. Post-update vehicle logs are required to establish actual warning
and camera-latency recovery; the separate SPI burst is not claimed fixed.
