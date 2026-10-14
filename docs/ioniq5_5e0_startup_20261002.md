# Ioniq 5 PE 5e0 startup after display-priority revert

Initial analysis on 2026-10-02, followed by the user-requested startup event
fix below. No running-device changes were made.

Full cereal decoding of device 7f419ed56030e135 route
`000005e0--7f516232b0--0/rlog.zst` confirms clean commit
`864355ef2d5e12cb01f3617bae10e5d8103dbbb3`, C4/mici, carrot-wip.
SHA256: `02aec2755b33131b6ef481d9b8504c8eb487d695b0bb64502e8465c027bf9413`.
Times are relative to the first carState, monotonic 59.115820861 s;
startup messages precede this origin. Coverage ends near +59.9 s.

## Displayed error and its source

The displayed fault is `radarFault/noEntry`, text `Radar Error: Restart the
Car` / `openpilot Unavailable`, from +29.639 to +32.354 s (the next alert
begins at the latter time). Repeated enable requests occur during startup.
The radarFault event already exists by +6.104 s, while model/radar output
is absent, but the no-entry text appears on an enable request.

- Model loading takes 24.8 s, completing at +24.348 s.
- First model inference takes 2,568 ms; the worker's run_model takes 2,562 ms.
  First modelV2 is published at +29.326 s. Startup complete is logged at
  +29.376 s.
- The first model input is frame 539, logged as 538 dropped frames relative
  to its initial counter. The next input is frame 596, a jump of 57 following
  the long first inference/startup loop. These two startup gap records are
  duplicated across logMessage/errorLogMessage, not four independent gaps.
- Only those two cameraOdometry messages are invalid. All subsequent model
  frame IDs increment by one (605 transitions).
- radarState has 19 invalid messages, +29.360 through +30.495 s; first valid
  at +30.551 s. Every published radarErrors flag is false, including
  canError, radarFault, wrongConfig and radarUnavailableTemporary.
- livePose is invalid/inputsOK=false on its first 22 messages, recovering
  at +30.696 s. sensorsOK and posenetOK remain true. First successful enable
  is +32.354 s; the readiness sound event follows at +32.562 s.

Source inspection explains the misleading fault label: radard sets its
message validity from input SubMaster all_checks(), independently of decoded
radarErrors. selfdrived maps invalid radarState to radarFault when neither
canError nor radarUnavailableTemporary is set. The eGPU settling guard
applies to generic communication alerts, not this earlier radarFault branch.
This is evidence of model-startup/readiness propagation into a radar-fault
message, not evidence of a hardware radar failure. The exact failing
SubMaster check inside radard is not logged here.

The first-frame latency is measured; this log does not identify exactly
which one-time GPU initialization/compilation operation consumed it.

## Revert applied; recurring camera issue absent in this window

All 31 process snapshots show UI nice=19 on core6 and camerad nice=0 on
core6. Thus the display-priority revert is active.

After +32 s, camera publication age from recorded SOF is:

| Camera | Mean | Maximum |
| --- | ---: | ---: |
| Road | 51.387 ms | 61.777 ms |
| Wide | 49.278 ms | 60.187 ms |
| Driver | 46.591 ms | 58.422 ms |

There are no later model input gaps or invalid pose messages. Post-startup
model runtime summaries after +33 s report inference mean 38.189 ms,
maximum 39.788 ms, and zero dropped frames. Core6 averages 84.184% after
+32 s versus 93.990% across the previous 5dc segment. This supports improvement
in the supplied post-revert window, but the windows differ in duration,
speed and workload; it is not a controlled before/after trial or proof of
long-duration recovery. This segment includes only about 29 seconds after
readiness.

## Separate recurring SPI errors

At +58.575 to +59.064 s, 12 SPI failure diagnostic records show NACK retries.
All recorded final results are nonnegative. Panda's SPI checksum count
increases from 2 to 18 (delta 16). This transport problem persists after the
revert and is distinct from the earlier startup radar alert.

All 5,989 carState messages retain canValid=true, canTimeout=false and no
temporary/permanent steering fault. The 537 pandaStates have no fault entries;
RX/TX buffer overflow, safety TX block and safety RX invalid counters remain
zero. No expected process stops or changes PID in 108 manager snapshots.
Successful retries do not establish that every wire deadline was unaffected.

Camera startup also logs three unique VIDIOC_CAM_CONTROL errno-19 messages
(duplicated across log channels) and initial requeue/synchronization messages;
the later continuous streams do not show a persistent camera failure.

Private reproduction script, summaries and reduced evidence are archived in
`.analysis/archive/2026-10-02/ioniq5-5e0-startup/`. Raw log remains on NAS.

## Requested correction

The user explicitly requested fixing the warning after the initial analysis.
selfdrived now separates an invalid radar aggregate from actual decoded
radar errors. During initial eGPU loading/settling, incomplete downstream
readiness raises selfdriveInitializing, retaining NO_ENTRY and card's existing
initialization gate. Initial pose input invalidity uses this same event only
when sensor and pose-sanity flags are healthy. Explicit radar/CAN/configuration
errors and pose sensor/sanity failures retain their failure events.

A per-process readiness latch ends the startup classification once model
dependencies are healthy or the system is enabled. Later input failures are
communication/localization faults, even if the existing five-second eGPU
settling interval has not elapsed. Running communication failures no longer
use that interval to suppress the generic communication check. The existing
model-loading flag and five-second settling timer are not extended.

Thirty-four focused tests pass, including the real state-machine transitions
with event categories extracted from the production event definitions. They
verify blocked enable during startup, recovery, subsequent failures, explicit
CAN immediate-disable and radar soft-disable, pose failures, and timer expiry.
Ruff passes. Windows tests isolate the production event blocks/state machine
from unavailable device-only imports; this is not a whole-process replay or
vehicle validation.

The change corrects readiness gating and the erroneous restart-car diagnosis.
It does not shorten the observed 24.8-second load/2.57-second first inference,
alter model/radar outputs or validity checks, or fix the separate SPI burst.
Only source/tests are published; this vehicle-specific report stays local.
