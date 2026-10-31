# Light speed eGPU / locationd and PR39184 review, 2026-10-31

The supplied October 30 recording confirms repeated input-invalid locationd
warnings while **Light speed runs on the USB AMD eGPU**. The complete model loop
exceeds its 50 ms camera period. Camera publications remain consecutive over
their available range, but modeld skips frames and invalidates the corresponding
camera odometry. This is not evidence of a locationd crash or Jetson failure.

## Evidence

The exact supplied NAS rlog was copied read-only and its SHA-256 matched the
source: `9009aeae43c4b7ad7a57b76f0e4776920ea861f472e10973b3f30b73f7e1c7db`.
Only the supplied segment exists for this route in the NAS vehicle directory.
Decode uses the full cereal schema. Raw data, Params and analysis stay private
under ignored `.analysis/`. The available service window is approximately
28 seconds; boot-time initData is not used as the segment-relative origin.
Incident source: `10bf0a56a263e5e6701b1ee7ff3a2374a2cd4e20`, device `mici`.
All 27 worker timing records identify Light speed PKL `33ba0928...86019a3`,
`gfx1200`, generic ONNX artifact, AMD warp and 1344 x 760 cameras. The published
paired runtime is tinygrad `d39227beebd0c3a340dc42b2cdc1e3db1c36aed9`.

| Recorded observation | Result |
| --- | --- |
| Road / wide camera publications | 549 each; IDs 1203 through 1751, no gaps |
| Camera odometry | 537; 19 one-frame gaps and exactly 19 invalid messages |
| ModelV2 | 531; 19 one-frame gaps, messages themselves valid |
| LivePose | 537; 44 inputsOK=false; sensorsOK and posenetOK true in all 537 |
| LiveCalibration | 107; five invalid envelopes, calibration status stays calibrated |
| SelfdriveState locationd soft-disable alert | 2469 samples, approximately 24.691 seconds |
| Process identity | Same modeld/locationd/camerad/calibrationd/sensord PIDs throughout |
| Sensor observation rejection logs | No timing-check or sanity-check warnings |
| Thermal status | Green in all 48 samples; CPU maximum 55.1 C |

Different service endpoints/counts must not be interpreted as an additional
producer failure. Alert duration is integrated between adjacent selfdriveState
samples; the alert was already displayed at the start of this partial segment.
It is not a count of independent failures or evidence of the preceding segment.

Timing means are weighted by frame counts in 27 diagnostic windows (540 frames;
windows straddle segment boundaries). These are host call durations including
waits/transfers, not isolated GPU kernel benchmarks.

| Stage | Mean ms | Maximum ms |
| --- | ---: | ---: |
| modeld inference call | 47.130 | 50.511 |
| modeld postprocessing | 3.548 | 4.239 |
| modeld complete loop | 51.723 | 60.709 |
| Worker input upload | 19.965 | 21.143 |
| Worker warp call | 3.200 | 4.064 |
| Worker model call | 4.591 | 5.676 |
| Worker output read | 13.211 | 13.650 |
| Worker recurrent-state checkpoint | 3.950 | 4.637 |

The worker uploads 3,244,544 bytes/frame. Inference near 47 ms does not establish
sustained 20 Hz operation: the complete loop misses 50 ms. The checkpoint
preserves recovery from non-finite recurrent outputs; deleting it is not an
equivalent latency optimization.

## Propagation into locationd

Nineteen modelCameraGap records each report one skipped frame; their current IDs
match all 19 invalid cameraOdometry IDs. `fill_pose_msg` marks odometry invalid
when `vipc_dropped_frames >= 1`. locationd requires valid cameraOdometry,
liveCalibration and carState inputs.

Calibration publishes at one fifth of the odometry rate. Five gaps coincide with
invalid calibration publications, extending input invalidity until the next
valid calibration is consumed. The 44 bad livePose samples form 19 episodes:
fourteen single samples and five six-sample episodes (about 0.27 s each from
first to last sample). Published-input alignment explains 39 samples directly.
The other five are the final sample of each calibration recovery episode:
valid calibration was published 17.8-21.7 ms before livePose, but subscriber
consumption order is unrecorded. This is consistent with one-cycle consumption
lag, not proof of exact scheduling order.

selfdrived raises locationdTemporaryError for inputsOK=false. Repeated short
invalidations and alert persistence explain the long displayed warning; 44 bad
pose samples do not mean 24.7 seconds of continuously bad pose. This supports
model-loop throughput as the initiating failure here, but does not separate the
new model, tinygrad, USB transfers and host scheduling. No equal-input runtime
A/B was performed. Resetting calibration or suppressing invalid-input warnings
would not address the demonstrated frame loss.

## Upstream revert applicability

[PR39184](https://github.com/commaai/openpilot/pull/39184), merged as
`afedebcfd4213a76b99b1f11b5310ed05cee04df`, reverts
[0152aab4](https://github.com/commaai/openpilot/commit/0152aab4a4db0971494847406bd875f938aee598)
([PR39176](https://github.com/commaai/openpilot/pull/39176)). It restores the
Chestnut/Flip Flop driving PKL, two upstream warp PKLs and tinygrad as a set:

- Driving PKL: `898b5cfd...f5cdfd` -> `f0667cf6...344a3a4`.
- tinygrad: `d39227be...36aed9` -> `02e69816...60b3f1`.
- No locationd source changes. The description gives no root-cause explanation;
  no human explanation is present in its returned comments.

Carrot's existing Flip Flop entry already pins the restored f0667cf6 PKL. Its
NAS precompiled.json still pins `02e698160eed1751738212144a7ea2cfb160b3f1` and
runtime archive SHA-256 `c7c58622652231359ecea1a4253b63e157d002242bf596ede1b853f941f937ac`.
The new Chestnut PKL and global tinygrad bump were never applied to this branch.
Carrot compiles its camera-layout warp under the selected package runtime;
it does not substitute either upstream warp PKL. A global cherry-pick of the
revert is neither necessary nor the correct way to repair Light speed.

[Light speed PR39173](https://github.com/commaai/openpilot/pull/39173) remains a
separate model, reviewed at `c7cbe5de6a9dae4bab032bc5faeb24a248c41f8f`.
Its 33ba0928 PKL stays paired with d39227. Swapping only tinygrad to 02e698 would
create an unverified serialized-graph/runtime pair. Retaining that pairing is
not a claim of acceptable eGPU performance: this recording demonstrates that
the installed combination does not sustain the camera rate.

## Disposition and validation scope

This review changes no runtime, model pin, locationd validity threshold, model
selection or vehicle setting. N16 and Flip Flop remain selectable; Flip Flop is
the already-packaged upstream-revert model/runtime alternative. Selection changes
require the normal parked reboot workflow; no remote change or cure validation
was performed on this vehicle.

Further Light speed eGPU work requires parked comparisons of the complete loop,
camera IDs, odometry validity and livePose, preserving numerical output/history
semantics and recovery. USB upload and state checkpoint are measured costs to
investigate, not proven removable overhead. A compatible upstream recompilation
can be evaluated as a new matched package.

Jetson uses ONNX/TensorRT, not the eGPU tinygrad runtime. This revert does not
change the ONNX. The supplied log cannot validate Jetson operation; TensorRT
build and real inference checks require Jetson powered on. No device writes,
restarts, CAN commands, hardware A/B or road acceptance were performed.

Validation consists of full-schema log correlation, exact source comparisons,
upstream diff review and read-only package-manifest inspection. Documentation
format and user-doc mapping checks run for this report-only change; software
unit tests cannot establish hardware latency and are not claimed as a fix.


## EV6 follow-up: nine segments and automatic failure capture

The owner supplied nine further EV6 segments (5-13) on the same incident source
10bf0a56. All nine full rlogs were copied from their exact NAS paths and their
source/copy SHA-256 values matched (110,415,270 compressed bytes total). The
available carState interval spans 539.952 seconds. The full cereal schema was
used again; per-file hashes and analysis remain in ignored private scratch data.

This device is also mici, running the same Light speed 33ba0928 PKL on USB AMD
with AMD warp. It is not Jetson. Segment 9 contains a fallback to the internal
ModelState backend; segments 10-13 are consequently **not Light speed results**.
The modeld and locationd PIDs remain constant across the transition.

| Segment | Active path | Invalid cameraOdometry | inputsOK=false livePose |
| --- | --- | ---: | ---: |
| 5 | Light speed eGPU | 6 | 6 |
| 6 | Light speed eGPU | 7 | 12 |
| 7 | Light speed eGPU | 12 | 17 |
| 8 | Light speed eGPU | 7 | 12 |
| 9 | Light speed eGPU, then internal | 2 | 2 |
| 10-13 | Internal | 0 | 0 |
| Total | | 34 | 49 |

Both cameras publish 10,800 consecutive IDs (6003-16802) without a gap, including
segment boundaries. ModelV2 and cameraOdometry each have 10,754 publications
(IDs 6001-16801), with 47 missing IDs: 34 isolated frame skips plus 13 IDs missing
across the timeout/fallback transition. All 34 invalid odometry IDs match the
ordinary modelCameraGap records. Three invalid calibration publications extend
the corresponding livePose episodes, giving 49 bad-input samples. SensorsOK and
posenetOK remain true in all 10,754 livePose messages; no sensor timing/sanity
rejection messages are recorded. All 988 device samples are thermally green;
maximum reported CPU temperature is 58.5 C. This is not an eGPU die-temperature
measurement or proof of hardware health.

Before fallback, frame-weighted completed timing windows give:

| Stage | EV6 mean ms | Earlier Ioniq5 mean ms |
| --- | ---: | ---: |
| Entire modeld loop | 50.268 | 51.723 |
| Inference call | 45.760 | 47.130 |
| Postprocessing | 3.500 | 3.548 |
| Worker input upload | 20.068 | 19.965 |
| Worker output read | 11.679 | 13.211 |
| Worker recurrent-state checkpoint | 3.983 | 3.950 |

EV6 is closer to the 50 ms budget but still has little margin and intermittent
frame loss. These are measurements on different devices/drives, not a controlled
A/B. Completed-window maxima exclude a call that times out without publishing
its timing sample. After fallback, segments 10-13 have mean inference times
24.0-24.6 ms and roughly 21 ms waiting for the next camera frame. Their complete
loops remain near 50 ms because they are camera-paced, not because all 50 ms
are processing. No livePose input failure recurs after the transition.

### New failure evidence

The rlog records `TimeoutError: precompiled eGPU worker timed out`, followed by
`eGPU model failed, falling back to internal GPU`, at 2026-10-31 09:11:52 KST
(monotonic 1271.640). The runtime response deadline is 150 ms. The observed gap
between published model messages is 667.913 ms (frame 10941 -> 10955), including
failure handling and fallback; it must not be reported as 668 ms of GPU compute.
The separate modelCameraGap entry reports 11 skipped camera inputs during
fallback, whereas the publication stream misses 13 IDs. These are distinct
counts because failed/preparation iterations need not publish an output.

The matching automatic egpu_error capture at 09:11:58 was found on the NAS,
read without contacting the vehicle, and source/copy hash matched:
`97f282c2670baa0b3d75ed84118f4eb9bf102d772a03a5f2412f5bbeabe547de`.
Its saved failure identifies the same model, process and incident timestamp:

- Last worker progress stage: `output_read`, worker iteration 10500.
- Stage age at the saved snapshot: 123.169 ms.
- Main worker state: sleeping, kernel wait channel `pipe_wait`.
- Artifact rejection is false; code classifies this timeout as transient-device.
  This classification is a recovery policy, not a diagnosis of a physical fault.
- USB device remains enumerated at 5000 Mbit/s, authorized, active, power control
  on. The captured kernel USB excerpt contains no incident-time disconnect/reset
  line. This does not rule out a transfer stall, cable/power problem or GPU fault.

**Stage-label limitation:** generic runtime marks output_read before output
readback, but does not update the stage during output validation or recurrent
checkpoint/save/synchronization. Its next marker is publish. Therefore the saved
output_read age cannot distinguish output download from those later operations.
Likewise pipe_wait is a kernel wait channel, not a Python/native stack identifying
which pipe or subsystem is responsible. Do not label this as a proven USB-read,
GPU-compute, checkpoint or tinygrad regression based on this snapshot alone.

### What would distinguish the remaining causes

The evidence now identifies two observed symptoms in the Light speed eGPU path:
insufficient sustained timing margin and a separate response-timeout episode.
They may share a cause, but the logs do not prove that. The clean internal period
is useful within-device evidence; it changes both model and execution backend,
so it cannot isolate Light speed weights from tinygrad or USB transport.

1. On the same parked device/eGPU, compare the existing matched Flip Flop package
   and Light speed package with the same input/settings and multiple minutes of
   recording. Compare whole-loop timing, tail delays, frame IDs, odometry/pose
   validity and failures. This isolates the package path better than another
   vehicle, but still changes both model and runtime.
2. To isolate tinygrad, use the same model and inputs with independently compiled,
   verified artifacts for each runtime, or matched upstream equivalents. Swapping
   only the tinygrad version underneath the existing PKL is not a valid test.
3. At the next timeout, separate output-read, validation, state-checkpoint dispatch
   and synchronization progress markers. Obtain the worker/child thread stacks,
   wait-channel/scheduler deltas and USB completion/error evidence at that instant.
   The existing automatic failure capture is useful and has already been retrieved;
   asking the owner to resend the same file adds no evidence.

Additional historical raw logs can show recurrence and model identity, but more
uncontrolled Light speed driving alone cannot separate these causes. No new drive
is required merely to reproduce this already-observed warning. No code/runtime,
vehicle setting, live device operation or CAN action was changed in this review.
