# OS04C10 exposure update / camera gap investigation, 2026-09-24

Status: parked A/B/A reproduced the original fault twice; grouped writes completed
the same 10-minute challenge without a gap. A separate 10-minute normal runtime
test with the driving model and DM also completed without invalidity or skips.
This is not a claim that every locationd or camera warning has the same cause.

## Historical comparison

The full cereal schema was used for the supplied Ioniq 5 logs and the available
September 11-24 comparison corpus. On the user's device 07b62e389ed26c81,
the latest fa0--3 fault followed ordinary radar work: the preceding second had
at most 20 liveTracks points, fast radar work at most 0.355 ms and longitudinal
planner work at most 6.006 ms. Radarcan had no pending carState backlog and
at most two pending CAN packets in the overlapping diagnostic windows. Normal
f9b--8 reached 32 points. Its proclogd peak was 57.898% of one CPU, similar to
58.379% during fa0--3. Earlier isolated wide gaps did not coincide with the
40-second PSS refresh. These findings do not exclude every workload interaction,
but do not support radar object overload or PSS scans as the common explanation.
The user's selected logs have no usable GPS fixes or Carrot position coordinates;
their physical location overlap cannot be established from those fields.
qcamera comparisons aligned using the first qRoadEncodeIdx timestamp show
nighttime roads with streetlights in f8c--6, f90--4 and fa0--3. The latest passes
bright roadside businesses. These views are consistent with illumination changes,
but do not establish identical locations or the wide sensor's exact field of view.

Four isolated wide-camera gaps have a more specific common pattern. The final
row is a separate Ioniq 5 C4, dongle 7f419ed56030e135, not the user's device.

| Segment | SOF gap (ms) | Prior exposure transition | Possible mixed value |
| --- | ---: | ---: | ---: |
| 00000f8c--3ae462b8c7--6 | 91.762 | 2299 -> 2308 | 2555 |
| 00000f90--96d7dcd525--4 | 101.038 | 2298 -> 2309 | 2554 |
| 00000fa0--1c7e2320b7--3 | 96.201 | 2273 -> 2349 | 2529 |
| 00000594--abf5912e57--7 | 102.044 | 2252 -> 2344 | 2508 |

All occur three software frame IDs after the update call inferred from the
cameraState exposure fields. `sendState()` records the previous commanded
exposure, then computes/writes the next command before publishing. These are
command values, not sensor readback. Normal logs also contain many such
transitions (for example 138 wide transitions in f40--5 without an isolated gap).
Therefore crossing alone is not sufficient evidence of a fault on each write.

## Concrete code hazard

OS04C10 exposure occupies byte registers 0x3501 and 0x3502. Existing code writes
the high byte before the low byte without group hold. For 0x08fa -> 0x0905,
an internal latch between these writes can see 0x09fa (2554), above the configured
maximum 2352 (VTS 2360 minus 8). This is a source-level race candidate, not a
direct measurement of the transient register contents.

The [manufacturer OS04C10 specification, revision 2.01](https://www.pdapply.com/upload/OS04C10-Product-Specification-CSP_Version-2-01_.pdf)
describes exposure registers and VTS-8 in section 5.6, group hold in 2.11-2.12,
and delayed manual launch in 2.13.2. Its group mechanism supplies a coherent
register set before the internal frame-boundary latch. The candidate records
the existing six exposure/gain bytes in group 0, ends the group, then launches
it at the sensor's vertical blanking point. Manual group mode is configured
at sensor initialization. Exposure targets, limits, gains, camera priorities,
model inputs and validity thresholds are unchanged.

## Parked reproduction on existing bt1 OS

The original 209b3c0c camera binary SHA256 was
`a85348d514fadf3afeb71ad94332c0ac3ef1baee16fbae99ead039ab9f5a3c8a`.
The existing CTRL_EXP_FROM_PARAMS diagnostic feature was enabled temporarily,
gain index 2 (1.125) held constant, and exposure alternated between 2298 and
2309 roughly every 250 ms. All values were within the existing sensor range.
The controller continuously required P, zero speed, controls disabled and
onroad state, and restored the exact binary and original Params afterward.

`trace04-exposure-old` triggered after 39.488 seconds:

- Driver-camera BOOT_TS interval: 95.670762 ms, consecutive software frame IDs
  843 -> 844. Road/wide maximum intervals stayed 53.529/53.472 ms.
- Driver exposure changed from 2298 at frame 841 to a reported command 2309
  at frame 842. The fault was at frame 844, matching the historical offset.
- The kernel's existing CSID hardware timestamp trace also had a 95.677344 ms
  interval for the affected context. The other two hardware streams remained
  within 49.996770-50.018021 ms. This rules out merely late userspace receipt
  for this reproduction; hardware timestamp gaps alone still do not expose
  the exact transient sensor register value.
- The preceding driver interval was 54.242 ms, followed by the long interval;
  all subsequent retained driver intervals returned to about 50 ms.
- This camera-only restart left modeld's old VisionIPC connection stale during
  the trial. Model/pose messages were absent, so this trial establishes a
  camera-path reproduction, not model/pose validity under a full driving load.
  Modeld was subsequently restarted and modelV2/livePose publication recovered.

The preceding 20-minute ordinary parked capture had no >75 ms gap, with a
58.280 ms maximum across cameras. A separate 10-minute DM-enabled capture
had 11,898 valid DM messages and no invalid cameraOdometry/livePose messages,
with maximum camera intervals 59.116 ms. Its end-of-capture DM byte-field JSON
serialization failed; metadata counters and kernel traces were saved, but its
full retained message series was not. DisableDM was restored to 2.

## Validation limits

- Moving-vehicle and C3 validation remain outstanding; parked C4 results are not
  proof of those conditions. AR0231 and OX03C10 register sequences are unchanged.
- Preserve the distinction between common all-camera scheduling jitter and
  the isolated sensor-stream gap. No claim about every earlier IFE error.
- The experimental bt2 kernel build passed C3 and C4 compilation with bt1
  userspace frozen, but it has not been installed for this reproduction.
  Do not attribute the exposure result to the Bluetooth fix or new kernel.

## Controlled A/B/A result

Only the camera exposure payload and its group-mode initialization changed.
All three trials used the same bt1 OS, gain index 2 and 2298/2309 commands.
These are camera-path trials; model/pose services were absent after camera-only
restarts and must not be described as full-runtime validity checks.

| Trial | Duration | Camera frames each | Result |
| --- | ---: | ---: | --- |
| Original `trace04-exposure-old` | Trigger at 39.488 s | Stopped on fault | Driver 95.670762 ms; CSID HW 95.677344 ms |
| Grouped `trace05-exposure-grouped` | 600.003 s | 12,000 | No >75 ms gap; maximum 57.112389 ms |
| Original repeat `trace06-exposure-old-repeat` | 133.002 s including trigger tail | Stopped on fault | Driver 95.652886 ms; CSID HW 95.656198 ms |

The repeat again had the 2298 -> 2309 command at reported frame 2713,
a 54.270303 ms interval at 2714, then the long interval at 2715. Both other
hardware timestamp streams remained within 49.996770-50.017969 ms in the trace.
The grouped binary SHA256 is
`aa9b4b9a7c39091c2bc5f6ae77c551f695726b5e86ad67424cbc8df291d26d1e`.
The old executable and Params were restored after each bounded trial.

`trace07-grouped-brightness` additionally alternated exposures 20 and 2000 every
five seconds for 60 seconds. After excluding the first 12 frames of each phase,
the actual image-derived median grey values were 0.222656/0.925781 (road and
driver) and 0.253906/0.925781 (wide). Thus the grouped command was not simply
ignored. All 1,202 frames per camera arrived, maximum interval 53.756 ms.
For example the wide exposure command appears at frame 281 and image-derived
grey changes at 284. Both telemetry fields describe the preceding update/image;
the inferred write at frame 280 affects image 283, consistent with the existing
three-frame automatic-exposure feedback delay.

The hazard exists in the September 11 comparison source as well: the sensor
files did not change between 490c391a08 and 209b3c0c. These results identify a
latent exposure-update defect and a reproducible fix, but do not establish which
recent workload/OS change, if any, increased its incidence. Location-dependent
brightness is a plausible trigger, not a confirmed geographic correlation.

## Normal automatic exposure and full runtime

After the grouped sensor sources and binary were installed, the device rebooted
on the original 19.8-carrot-bt1 OS. Normal automatic exposure was restored;
CameraDebugExpTime/Gain were absent and there was no diagnostic launcher.
Model, pose and all camera services were confirmed alive/valid before collecting.
DM was temporarily enabled and restored to DisableDM=2 in the controller's finally.

`trace08-grouped-full-runtime` ran 600.039 seconds while parked with UsbGpuActive=1:

- All three cameras: 12,000 messages each; maximum road/wide/driver SOF intervals
  57.532827 / 57.536628 / 57.540014 ms, no >75 ms interval.
- ModelV2, cameraOdometry, livePose and driverStateV2: 12,000 messages each.
  Full-run invalid-message/pose-flag/model-frame-gap counters were empty.
- Accelerometer: 55,176 samples, maximum event-minus-sensor age 34.972979 ms.
  Gyroscope: 54,421 samples, maximum age 35.796381 ms.
- The collector retained a bounded tail of message rows and kernel events;
  the counts, maxima and invalidity counters above cover the whole 600 seconds.
  Synthetic exposure challenge and normal automatic exposure are separate tests.

The C++ regression evaluates the actual generated register payload at every
possible byte-boundary latch for 972 exposure/gain transitions. It checks coherent
old/new exposure, the permitted range, matching gain channels and final values.
It models the documented sensor latch contract, not sensor electronics.
Both versions compiled with the vehicle's clang++; original writes failed the
gain-channel coherence check, while the grouped payload passed all 972 cases.

## Separate IFE error still unresolved

`00000f8d--e6ec5e5328--3` at +30.936 s is different: wide exposure stays at
82 with gain 1.0 across the incident. The kernel records sync state 3 and the
application's IFE wait fails in 0.23 ms, followed by recovery and a 204.240 ms
wide gap (three missing frames). In the deployed kernel, state 3 is
`CAM_SYNC_STATE_SIGNALED_ERROR`: `cam_sync_wait()` returns -EINVAL after the
completion has been signalled. This is not a 100 ms wait timeout or proof that
the CPU merely delayed camerad. ISP bubble/error/flush paths can signal this
state, but the retained historical log does not identify which path initiated it.
The request-state dumps and subsequent CRM warnings alone do not resolve that
ambiguity. Do not claim this separate incident is fixed by exposure grouping.
Its preceding second also had ordinary radar work: at most 19 points, fast radar
maximum 0.297 ms and longitudinal planner maximum 5.113 ms. Coarse CPU samples
cannot rule out a short kernel scheduling delay, but object-count overload is
not supported. The following segment had a separate simultaneous ~75 ms
all-camera jitter near its end with no model drop; do not conflate it with the
204 ms single-stream recovery.

Docs-Not-Needed: Internal camera diagnostics and sensor correctness; no user setting changes.
