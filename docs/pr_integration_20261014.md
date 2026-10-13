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

The nav port also passes 750 focused desktop tests. Its pre-existing absence of
the wip Bosch context adapter is preserved. The first built Linux integration run
found one test-only logging incompatibility: carlog emitted the startup warning
without propagating it to pytest's root capture. Check the logger call directly;
do not alter the receiver or logging policy to satisfy the test.

Native x86 and ARM each pass 207 kernel/CAN tests, 586 subtests and 794 radar
regressions. The NAS workflow passes its 291 + 40 + 254 + 1,087 receiver/replay
checks. Wiki commit `e4f2435` publishes 26 Korean/English manual descriptions;
the generator validation and subsequent Wiki workflow pass.

Feature commits are wip `f513c8f8bb` and nav `57fc2df0b4`; test portability and
Wiki-publication notes are wip `504a902155` and nav `099b4681d8`. All are
single-parent commits. Existing uncommitted edits were preserved, including the
independent local NNFF catalog addition and its rebased count tests.

The [wip full Linux run](https://github.com/ajouatom/openpilot/actions/runs/38486400559)
passes native compilation, 598 new integration checks (including actual radarcan
IPC/recovery), 726 Bosch checks, 116 UI checks and 1,035 model/monitoring/Bluetooth
checks. The separate startup, DM cadence, speed and update-lifecycle jobs pass.

The [nav full Linux run](https://github.com/ajouatom/openpilot/actions/runs/38486410678)
also passes: native compilation, 597 new integration checks, 116 UI checks and
1,040 model/monitoring/Bluetooth checks, plus the separate lifecycle jobs. The
one-test integration-count difference is wip's existing Bosch context test.

The automatic NAS updater reports `504a902155e47f6921b1c67e90f22cd8f61efa2b`.
Public health and an existing upload-result page respond successfully; its actual
recalculated payload has 1,199 frames and source version `ef933ee9237806ede338`,
matching the updater's verified replay. Retain the private route and response hash
in `.analysis/archive/2026-10-14-pr-integration/`, never in the public guide.

Synthetic and recorded-input tests are not closed-loop road validation.
