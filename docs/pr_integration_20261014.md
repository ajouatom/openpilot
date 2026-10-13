# PR 547 and RAV4 PR 548 integration — 2026-10-14

The user accepted all remaining features, superseding the CPU-based deferrals in
[the October 13 review](pr_review_20261013.md). Apply the same feature changes to
carrot-wip and carrot-nav with single-parent commits, preserving each branch's
model/runtime/navigation choices. This task does not install software on a vehicle.

## Accepted code

- [PR 547](https://github.com/ajouatom/openpilot/pull/547), joongyu01,
  `67de82cd003a78a5160c67e65372408f5fa1dadf`: stock-navigation preview matching,
  eligible deceleration cancellation, section average-speed allowance, CAN-FD
  lever counters and turn gesture, queued lane changes and signal holding, and
  the mode-3 adjacent-SCC exclusion. Keep the earlier latActive cluster guard.
  The author's latest revision removes Turn/Straight speech; no speech is added.
- [PR 548](https://github.com/ajouatom/openpilot/pull/548), Mlnege,
  `c8980921e0207773159f3f7907d35356c16b7e17`: exact 2022 RAV4 RADAR_ACC ARS510
  reception, observed PCM speed synchronization in direct-longitudinal mode 0,
  and independent display points for standard-screen radar mode 3.
  Preserve the vendored decoder and MIT license, including in the NAS image.

New opt-in settings retain their contributed defaults. Mobile-zone filtering and
lever queue/signal behavior include changes independent of the new toggles; an
all-toggles-off state is not claimed to reproduce the previous revision exactly.
Schema/Params changes require a native rebuild after updating.

## Review corrections and boundaries

The original mode-3 one-second exclusion remembered only a timestamp. SCC often
reuses ID 0, so rejecting an adjacent car could suppress a new uncorroborated lead.
The hold now requires the same source/ID, a nonnegative gap no longer than one
second, predicted-range agreement within 2 m and relative-speed agreement within
3 m/s. Missing SCC/path and leaving the fallback reset it. These continuity checks
do not prove identity when two objects have indistinguishable kinematics.

Mode 3 remains front/vision first, then SCC unless physical evidence identifies
an adjacent body, then vision. Modes -1/0 retain unconditional longitudinal SCC.
The RAV4 display decoder cannot supply control leads. Other Toyota receivers,
Panda, ECU command policy, PCS/AEB and actuator limits are unchanged.

The new algorithms add CPU work; no vehicle CPU percentage or deadline guarantee
is established. RAV4 runs a second bounded display decoder. Navigation/lever rules
use scalar state and bounded point/preview scans, without new neural inference.
Section distance scaling remains the author's 1.015 calibration assumption; it is
not a measured calibration for every car or proof of camera-average compliance.

## Verification

- Windows: 769 focused tests pass, including real Cap'n Proto schemas, CAN packing,
  ARS assembly/CRC/recovery, PCM priority, navigation/lever rules, display isolation,
  SCC target changes and existing lead policy, settings schema and Wiki generation.
  Unbuilt Params IO is blocked or explicitly isolated by test fixtures; hardware
  selection is PC for pure controller tests. Native messaging is not mocked into
  a claimed IPC test.
- Added the contributed integration tests to the full Linux build workflow and
  the new SCC regressions to x86/ARM native-kernel checks.
- KO/EN guides and setting-level Wiki descriptions cover the new behavior. New
  MANUAL text is generated/validated with the current catalog.

Linux CI, publication and the automatic NAS replay verification are recorded after
completion. Synthetic and recorded-input tests are not closed-loop road validation.
