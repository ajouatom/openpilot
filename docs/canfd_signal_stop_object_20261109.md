# CAN-FD signal-stop object and cone trial (2026-11-09)

The owner reports recurring creep during a no-lead signal stop in their CAN-FD
vehicle and requests a stationary object about 3 m ahead in the SCC output,
shown as a cone on the Carrot screen. This is separate from the Casper CAN/FCA
fault investigation. The latest failed-stop log was deleted; archived earlier
stock and failed-stop evidence remains available. The latest recurrence's cause
cannot be identified or a physical remedy verified from that older evidence.

## Original comparison

Full-cereal re-decoding of archived Ioniq 5 PE stock segments 1067/4 and
1069/1–2 confirms openpilot longitudinal control was off. Their bus-2 SCC
StopReq=1 packets total 1,138 (966/24/148), all with object state 2, distance
3.4–4.4 m and finite relative speed -0.6 to +1.5 m/s. Both acceleration requests
are zero; Lower band is 0.04. These segments include one stop spanning a segment
boundary; they are not three independent trials.

Earlier failed no-lead signal-stop segment 1116/1 has 680 TX-echo StopReq
packets, all with object state 0, distance 204.6 m and relative speed 239.4 m/s
(no-object encoding), both acceleration requests zero and Lower band 0.20.
The subsequent October 23 change already makes the no-object state 2 under
final StopReq, but leaves the no-object range/speed encoding in place.
The new trial fills those object fields. It does not change Lower band, jerk,
aReq, StopReq admission or the stopping state machine, and does not assert that
any single difference is the ESC's acceptance condition.

## Implementation

- Scope: Hyundai CAN-FD camera-SCC output, with active openpilot longitudinal
  control. Classic-CAN Casper and non-camera CAN-FD builders are unchanged by
  the stop-object trial.
- Controlsd requests the object only for fresh valid planner/radar inputs,
  shouldStop and signal-stop state 3/5, with no reported leadOne/leadTwo or
  planner hasLead, drive gear and no pedal/parking/auto-hold interlock.
- The camera-SCC builder applies it only after the existing stop controller
  and final ACCMode=1/StopReq=1 admission. All ego/wheel speed samples must be
  finite and within the existing 0.7 m/s initial stop-entry region.
- Final object fields become distance 3 m, lateral position 0, relative speed
  **-vEgo** (stationary absolute object) and HUD_LEAD_INFO/SCC_ObjSta=2.
  Real lead fields retain priority. Release, invalid inputs, interlocks,
  missing source packet and movement outside the low-speed region clear it.
- No synthetic radarState/liveTracks/model/MPC observation is created. The
  original RX SCC dictionary is never modified.
- `carOutput.actuatorsOutput.stopObjectActive` reports inclusion in the generated
  SCC packet. It is not ECU acceptance, TX acknowledgement or physical hold.
  C3/C4 Carrot renderers draw an orange/white cone from that reported state,
  requiring fresh same-session output/control/vehicle/radar data and clearing
  immediately for a real lead or release. Its ground projection is 3 m; near
  points below the camera crop are clamped to the lower screen boundary.
  This is Carrot UI rendering; the OEM cluster chooses its own icon.

## Validation

Recorded-input comparison sorts publications by their original monotonic timestamp
and reconstructs 2,998 builder calls from archived 1116/1
with the same recorded vehicle, radar, planner and command inputs on both sides.
There are 682 admitted requests, with 681 final-packet changes (46.371–59.970 s).
Those packets change only distance, relative speed and recomputed CRC; 2,317
packets remain byte-identical. The existing state-only trial already supplies
state 2. Both sides retain exactly equal acceleration outputs. The replay
reconstructs wrappers/jerk from logged inputs and is not a bit-exact reproduction
of the old runtime or a closed-loop vehicle simulation.

Desktop tests cover real CAN pack/unpack, real-lead preservation, no RX mutation,
signal-state scope, final interlocks, missing/invalid/stale inputs, departure,
fault event/state-machine handling, full state_control with AlwaysLateral, and
cone expiry/drawing. The actual cone drawing routine was inspected using a
synthetic road background; this does not verify in-car placement/readability.
Windows tests isolate hardware/Params/IPC only; production schemas, policy,
controllers and CAN codec are used. Platform-native checks are run in Linux CI.

The combined desktop run passes 465 checks; the separate Wiki generator and
validator suite passes 25 checks, and KO/EN settings documentation validation
passes. The AlwaysLateral catalog/guides/Wiki explain confirmed-fault inhibition.

Linux Signal integration 42554434320 passes the existing 1,960 checks plus 362
subtests (40 platform-specific skips), 131 Python-backend checks and 216 new/focused
fault/stop-object/UI checks. Final cone crop-boundary revision b8f7f3b405 also passes
Signal integration 42554894007. eGPU compatibility and native CPU checks pass for
the control revision. These results do not replace vehicle validation.

The normal Carrot Routes image workflow 42554434278 published the shared control/
schema revision 7d337b0549. The NAS scheduled updater reports updated/no error;
independent public health and result-page requests return HTTP 200, and the
1,196 served radar frames (fingerprint `2d00544cf2f72147cda1`) match the fresh
deployment-probe hash. The cone-only follow-up does not change radar replay code.

Full-build workflows 42554434257 (control revision) and 42554893754 (final cone
revision) compile successfully and pass the control-loop, geometry/UI, radar and
other preceding integration steps. Their final model/monitoring/Bluetooth group
has 1,154 passes and four `ModuleNotFoundError: tools.jetlink` failures in the
Bluetooth API/B1 tests. The same four test names and exception occur on the
unchanged baseline 0df83618f1 / run 42529186127. They remain unresolved; the full
workflow is not green. The focused native workflow on final code is green.

No vehicle installation, CAN actuation or road test was performed. Delivery is
normal comma Git Pull and full restart/reboot. No new model or Jetson runtime
bundle is part of this change. Physical stop/hold acceptance remains unverified.
Private archived source logs and reproduction scripts are under
`.analysis/archive/2026-11-09/stop-object/`; raw evidence is excluded from Git.
