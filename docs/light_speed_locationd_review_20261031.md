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
