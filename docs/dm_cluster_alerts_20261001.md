# DM startup notice and Hyundai/Kia cluster warnings — 2026-10-01

The user reported a DM-camera error at startup and requested showing DM attention
warnings on Hyundai/Kia CAN-FD clusters, then extended the request to classic CAN
where the DBC supports it. The current incident's device/route and exact screen
were not supplied; its physical fault status is therefore not established.

## Startup presentation

`cameraUnavailable` includes unavailable driver inference, driving model and
calibration inputs, followed by the existing two-second recovery qualification.
It does not identify a failed camera sensor. During the first 30 seconds of
selfdrived's loop, suppress only the fallback *notice* until camera monitoring
has first become usable. After first recovery, loss uses the existing notice
behavior immediately, including the existing three-second creation delay.
If startup never recovers, the event becomes eligible at 30 seconds and its
normal creation delay still applies. The interval is not renewed by DM toggles.

Camera availability, interaction fallback, attention warnings, terminal alerts,
lockout, force deceleration and process/camera fault checks are unchanged.

Historical verification uses full-schema NAS rlog
`HYUNDAI_IONIQ_5_PE 07b62e389ed26c81/00000ff1--64803a8344--0`, commit `250f14ed`.
This is the previously investigated ff1 startup, not an identification of today's
unspecified incident. It contains 1,217 driverMonitoringState, 1,203
driverCameraState and 5,976 selfdriveState messages. Camera fallback clears at
16.9835 seconds relative to first carState; the old notice appears on 797
selfdriveState messages. Replaying the new notice predicate yields no eligible
notice samples. First recorded selfdriveState approximates the loop-time origin.
Earlier analysis of the same log found consecutive camera frames and delayed
model/calibration readiness; see `jetlink_ff1_investigation_20260929.md`.

The same segment has no stock DAW alert: all 1,228 camera-bus 0x161 frames have
ALERTS_2=0, DAW_ICON=0 and SOUNDS_1..4=0. Thus it verifies startup behavior but
does not establish which stock DM popup was experienced or blocked. The source
code independently confirms that the existing filter clears ALERTS_2=5 and its
DAW icon, among other stock warning values.

## Cluster mapping

`CarControl.HUDControl.driverMonitoringAlert` carries level 0/1/2/3 only from
valid, alive, frequency-qualified, enabled driverMonitoringState. Disabled or
unhealthy monitoring exports zero; absent fields in historical logs default to
zero. This is independent of generic `steerRequired`, so unrelated steering
warnings cannot masquerade as a DM warning. AlwaysOnDM remains eligible while
disengaged. Warning timing and the existing openpilot sound stages are unchanged.

| Existing output path | DM stages 1/2 | DM stage 3 |
| --- | --- | --- |
| CAN-FD ADRV_0x161 | DAW_ICON=1, ALERTS_2=5 (consider a break) | DAW_ICON=1, ALERTS_2=9 (take control immediately) |
| CAN-FD LFAHDA_CLUSTER fallback | HDA_InfoPUDis=5 (hands-off popup) | Same visual popup |
| Classic SEND_LFA LFAHDA_MFC | LFA_SysWarning=5 (orange hands-on) | LFA_SysWarning=6 (red hands-on) |
| Classic Santa Fe without SEND_LFA | LKAS11 SysWarning=4 | LKAS11 SysWarning=5 |
| Classic Optima G4/G4 FL | Stage 1 remains device-only; stage 2 uses existing LKAS11 value 4 | LKAS11 value 4 |
| Other classic LKAS11 paths | Existing generic SysWarning value 3 | Same generic warning |

CAN-FD 0x161 changes are applied after the existing suppression filter to fresh
copies, preserving the input snapshot. Other surviving popup identities remain;
stock collision/takeover identities 7/8/9/10/14/21 and their stock sound fields
take precedence during DM alerts. No new CAN-FD sound request is added. 0x1e0
fallback is used only when the existing controller sends that message and no
0x161 display path is active; nonzero primary/secondary stock popups are retained.
No new message is fabricated when its stock snapshot is absent. HDA2 stock-long
configurations without an existing permitted cluster sender are not expanded.

Classic LFA warning enums are defined in `hyundai_kia_generic.dbc`; older LKAS11
values reuse the existing port's warning mappings/comments. Optima's known value
4 includes a beep, so it is not newly requested during silent DM stage 1. Classic
warning display state may become 3 to allow a disengaged AlwaysOnDM warning;
torque, actuation request, fault, lane departure fields and checksum algorithm
are unchanged. Actual display acceptance and implicit sounds remain unvalidated.

`LKAS12.CF_LkasDawStatus` and `CF_Lkas_Daw_USM` exist but lack decoded enum
meanings; no guessed values, new LKAS12 transmitter or Panda allowlist changes
are introduced. CAN-FD CAMERA_SCC retains the existing original-RX-paced Panda
cluster forwarding contract and its previously required firmware.

## Validation

- 335 desktop tests: DM notice/HUD qualification, both CAN-FD display paths,
  stock popup and sound precedence, CAN checksums/counters, classic warning
  mapping, cluster/lead/fault regression and handover-controller regression.
- 269 existing monitoring, traffic context, parking reset, daemon and wheel-touch
  tests pass with the existing Windows native IPC/Params/hardware adapters.
- New modules/tests pass Ruff. Actual CAN packer, DBC and cereal schemas are used.
- Local replay script, extracted summary and test runner are archived under
  `.analysis/archive/2026-10-01/dm-alerts/`; raw log copies are reproducible from NAS.

These tests do not validate physical cluster rendering, chime loudness, driver
camera hardware or vehicle response. No setting or monitoring budget changed;
localized user guides are outside this display/notice correction.
