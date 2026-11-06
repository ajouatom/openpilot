# Comma Pedal integration and diagnostics

The shared Comma Pedal protocol is opt-in through `CommaPedalEnabled` under Settings → Vehicle and hardware → Device hardware. The persistent integer defaults to 0 and uses the existing typed Params writer. A saved change is read at the next vehicle-parameter initialization. It does not alter an active session's parsers or Panda safety configuration.

The source reference is StarPilot `98f8443122909a86c5a5fd628337c4077cd1f957`; the Carrot baseline before the previous pedal port is `604bc0a22733e648ec05b268da1ffaae7db52a69`. The four previous port commits were removed in the working copy before this integration. No remote history rewrite is required.

## Protocol and vehicle configuration

`opendbc_repo/opendbc/car/pedal_interceptor.py` handles the wire codec, state and command cadence. The separate `pedal_interceptor.dbc` uses Ray command factors/offsets `(0.336, -88.704)` and `(0.166, -82.502)` after PR 570. Sensor factors/offsets are `(0.672, -177.408)` and `(0.332, -165.004)` with physical ranges 0–255. TX is 0x200/DLC6 and RX is 0x201/DLC6 on bus 0, with two 16-bit tracks, ENABLE/STATE, a 4-bit rolling counter and CRC8 (initial 0xFF, polynomial 0xD5, reversed payload order excluding the CRC byte). Reception is 50 Hz and command transmission is 25 Hz from the 100 Hz controller loop. Enabled commands pass `amount * 255` directly to the packer; disabled commands omit both tracks to retain raw-zero/disable heartbeat. Physical driver thresholds still use the raw received bytes.

The supplied Ray calibration is in `car/hyundai/pedal_interceptor.py`. Its target caps are `[0.55, 0.55, 0.65, 0.70]` and normalized rate limits are `0.02` up and `0.06` down. PR 570 doubles the unrounded command increment above raw neutral 264/497 through the DBC conversion. Consequently, raw increments per update also increase. At target 0.70, the rounded maximum is 795/1572 and the Ray Panda profile enforces those limits. Physical input thresholds and Panda track limits belong to that explicit vehicle profile. They are not protocol-wide defaults. The existing Carrot longitudinal PID, state transitions, lateral controller, LFA display and driver-monitoring indications remain in place.

The Hyundai adapter requires `CC_ONLY_CAR`, an explicit configuration, the opt-in parameter, a classic non-legacy/non-camera-SCC path, a 6-byte pedal sensor fingerprint on bus 0, and no observed SCC11/SCC12 on buses 0 or 2. The independent device parser is created only on that path. LFA and LDA markers are not common protocol prerequisites. LABEL11.CC_React is OEM Cruise ready; raw bit 35 is OEM CC engaged. Pedal control is allowed only when both are off and the cruise-state message is present and fresh. OEM Cruise ready/engaged blocks positive pedal commands in both the controller and Panda; OEM cruise is not automatically canceled. Carrot's separate main-switch/controls-initialization admission remains in effect.

Other eligible `CC_ONLY_CAR` vehicles can supply their own target, calibrated DBC, thresholds, CAN adapter and explicit safety profile to reuse the protocol. Road-validation status is separate from configuration availability. The current supplied configuration is Ray EV; there is no automatic Ray fallback. Adding a profile requires the corresponding vehicle safety implementation and tests, including message-ID and bus conflicts. Honda, Toyota and GM interceptor DBCs have different units/calibration and retain their existing behavior.

The ordinary vehicle paths, shared safety core, common longitudinal state machine, Hyundai common/CANFD safety and forwarding policy are unchanged from the Carrot baseline. The new checksum/counter registration is limited to `pedal_interceptor.dbc`. New pedal safety permission does not reuse SCC TX auto-authorization and cannot bypass Carrot's LONG/ALLOW_DEBUG build policy. Firmware on the pedal device and voltage calibration are not changed.

The existing generic `LFAHDA_MFC` definition is sufficient; no dedicated LFA DBC is required. The pedal DBC remains separate because Hyundai EMS20 and other vehicles use the same CAN IDs for different messages. Packaging uses the existing `**/*.dbc` package-data rule, with no dedicated MANIFEST.in include. Packaging a file, importing a module and creating a live parser are separate operations.

## Validation status

The owner reports successful vehicle operation with StarPilot. That report does not establish vehicle operation with this Carrot integration. The reference target/codec comparisons, unit tests and before/after safety comparisons do not substitute for native Params integration, device firmware verification or a road test. The reported earlier “Waiting to start” condition was a user-setting issue and is not treated as evidence of a code defect.

This pedal path commands accelerator tracks. Hydraulic braking, regenerative response and stationary holding must be evaluated against the actual vehicle configuration; a successful pedal command does not establish all those behaviors.

## Diagnostic sequence

Compare matching StarPilot and Carrot logs with the commit, settings and vehicle/harness configuration recorded. Leave observations unknown when logs were not collected.

| Stage | Observations | Distinction |
|---|---|---|
| Hardware | Panda board/ignition, USB/SPI health, firmware version/signature versus expected image, CAN errors/bus-off | A pedal is a CAN node; USB enumeration alone does not prove detection. Physical Panda CAN health indices are not necessarily logical CAN `src` numbers. |
| CAN reception | Bus 0 0x201/DLC6, approximately 20 ms intervals, raw tracks/STATE/counter/CRC, required CLU11/TCS13/wheel-speed inputs | Missing device, wrong bus/DLC, error state, bad validation and stale reception are different failures. Check actual ID conflicts. |
| Identification/settings | Fingerprint/FW candidate, selected vehicle/flags, saved `CommaPedalEnabled`, native Params availability | A web save or memory-fallback success does not prove native Params persistence. |
| CarParams | Pedal capability, OP longitudinal, pcmCruise, passive/dashcamOnly, final safety model/param and alternativeExperience | A newly saved value can differ from the active session's CP. Ray's new safety profile is 0x6005; compare semantic bits across forks. |
| Vehicle/pedal state | carState canValid/canTimeout, gasPressed/brakePressed/accFaulted, LABEL11.CC_React (Cruise ready)/bit 35 (engaged), raw pedal STATE and last valid age | Cruise ready must be off for pedal control; OEM cruise takes priority while ready/engaged. Missing/stale cruise status also blocks pedal control. Internal parser/state objects are not serialized carState fields. Derive them from raw CAN or inspect an offline replay. |
| Command generation | selfdriveState/onroadEvents, carControl enabled/longActive/override, actuator acceleration, set speed, sendcan 0x200 | An intentional zero, disabled upper controller, missing output path and actual device failure require different remedies. Check 25 Hz, counter wrap, disabled raw-zero tracks and CRC. |
| Panda | Actual safety model/param, controlsAllowed, RX-check health, relay fault, TX-blocked counters, returned/rejected frames | A message queued in sendcan is not proof of Panda acceptance or transmission. Check the installed H7 image's signature. |
| Device response | Continued 0x201 state/input feedback, native E_EMS11 response and owner observations | Do not infer accelerator, braking or stop-hold performance without corresponding measurements. |

Use the full cereal schema when checking the presence of `can`, `sendcan`, `carParams`, `carState`, `carControl`, `controlsState`, `selfdriveState`, `onroadEvents`, `pandaStates` and `deviceState`. A reduced CAN-only schema can omit other services. Logging-related hardware settings can also affect available evidence.

## Panda build connection

The repository SConstruct calls `panda/SConscript` and `panda/board/SConscript`. `panda/board/main.c` includes the safety root through the existing `opendbc` link to `opendbc_repo/opendbc`, then `safety/safety_hyundai.h`. `panda/firmware_version.py` hashes safety source content. `pandad.py` compares the selected image's signature, while `panda_safety.cc` applies CP.safetyConfigs.

After an authorized installation, confirm both the built H7 signature and the actual device signature/configuration. This integration does not replace the Panda tree, bypass firmware verification or modify the pedal-device firmware.


## Wip integration review on 2026-11-03

PR 569 head `01b8bf5dda2b38a0015f78f4152f7fdb9bc23de8` was reviewed against
current Wip `36fbbbae3afa124f6d16c7ca6fe5122434a4ebe7`. A 2,568-case parameter
comparison covers all 107 Hyundai platform entries, toggle OFF/ON, absent/present
pedal, SCC-present fingerprints and four CameraSCC/radar configurations. 2,566
results match the baseline, including the same 24 existing initialization errors.
Only the two expected Ray EV, pedal-present, toggle-ON, eligible classic-CAN cases
select the new profile. Unsupported vehicles retain their previous CarParams.

The merged Python pedal, settings and current CCNC/ALT2 tests pass 438 cases.
Signal CI includes the production Panda pedal checks, switching from a pedal
profile into ordinary Hyundai/legacy/CAN-FD profiles, and both CAN backends.
The recently added isolated CameraSCC MAIN request remains covered by its tests.

The contributor reports limited acceleration on their test vehicle. This review
preserves the submitted Ray calibration and does not resolve or validate that
vehicle symptom. No vehicle writes, settings activation, pedal transmission or
hardware installation were performed during this integration review.


## PR 570 review on 2026-11-05

Reviewed head `5f6712c422e63e27e1d7c2c633b9d3263e53295b` against Wip
`90bd49d284`. The final diff changes command scaling, the explicit Ray output
bounds and regression tests. Eligibility, sensor conversion and driver thresholds
retain their existing definitions. The contributor reports successful operation
on a 2024 Ray EV after this change. Local review does not measure vehicle
acceleration or establish compatibility with other pedal firmware calibrations.

The Windows pedal, Hyundai button, CameraSCC driver-intent and ALT2 regressions
pass 221 cases. Native Panda checks run through Signal CI, including expanded
limits, over-limit rejection, track correlation, OEM cruise priority, sensor faults,
brake and driver override, and profile switching. Vehicle installation and
physical pedal operation are outside this code integration.
