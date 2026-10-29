# Comma Pedal integration and diagnostics

The shared Comma Pedal protocol is opt-in through `CommaPedalEnabled` under Settings → Vehicle and hardware → Device hardware. The persistent integer defaults to 0 and uses the existing `risk: high` caution badge and typed Params writer. A saved change is read at the next vehicle-parameter initialization. It does not alter an active session's parsers or Panda safety configuration.

The source reference is StarPilot `98f8443122909a86c5a5fd628337c4077cd1f957`; the Carrot baseline before the previous pedal port is `604bc0a22733e648ec05b268da1ffaae7db52a69`. The four previous port commits were removed in the working copy before this integration. No remote history rewrite is required.

## Protocol and vehicle configuration

`opendbc_repo/opendbc/car/pedal_interceptor.py` handles the wire codec, state and command cadence. The separate `pedal_interceptor.dbc` describes raw values: TX 0x200/DLC6, RX 0x201/DLC6, bus 0, two 16-bit tracks, ENABLE/STATE, a 4-bit rolling counter and CRC8 (initial 0xFF, polynomial 0xD5, reversed payload order excluding the CRC byte). Reception is 50 Hz and command transmission is 25 Hz from the 100 Hz controller loop. Disabled output continues the zero/disable heartbeat.

The supplied Ray calibration is in `car/hyundai/pedal_interceptor.py`. Its target function, command conversion, command cap and rate values are retained from the reference. Physical input thresholds and Panda track limits belong to that explicit vehicle profile. They are not protocol-wide defaults. The existing Carrot longitudinal PID, state transitions, lateral controller, LFA display and driver-monitoring indications remain in place.

The Hyundai adapter requires an explicit configuration, the opt-in parameter, a classic non-legacy/non-camera-SCC path, a 6-byte pedal sensor fingerprint on bus 0, and no observed SCC11/SCC12 on buses 0 or 2. The independent device parser is created only on that path. LFA and LDA markers are not common protocol prerequisites. Ray's ordinary cruise state uses LABEL11.CC_React and raw bit 35 of LABEL11; reading the latter does not change the shared Hyundai DBC. Carrot's main-switch/READY admission remains in effect.

Other vehicles can supply their own target, conversion, thresholds, CAN adapter and explicit safety profile to reuse the protocol. Road-validation status is separate from configuration availability. The current supplied configuration is Ray EV; there is no automatic Ray fallback. Adding a profile requires the corresponding vehicle safety implementation and tests, including message-ID and bus conflicts. Honda, Toyota and GM interceptor DBCs have different units/calibration and retain their existing behavior.

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
| Vehicle/pedal state | carState canValid/canTimeout, gasPressed/brakePressed/accFaulted, LABEL11 cruise bits, raw pedal STATE and last valid age | Internal parser/state objects are not serialized carState fields. Derive them from raw CAN or inspect an offline replay. |
| Command generation | selfdriveState/onroadEvents, carControl enabled/longActive/override, actuator acceleration, set speed, sendcan 0x200 | An intentional zero, disabled upper controller, missing output path and actual device failure require different remedies. Check 25 Hz, counter wrap, disabled raw-zero tracks and CRC. |
| Panda | Actual safety model/param, controlsAllowed, RX-check health, relay fault, TX-blocked counters, returned/rejected frames | A message queued in sendcan is not proof of Panda acceptance or transmission. Check the installed H7 image's signature. |
| Device response | Continued 0x201 state/input feedback, native E_EMS11 response and owner observations | Do not infer accelerator, braking or stop-hold performance without corresponding measurements. |

Use the full cereal schema when checking the presence of `can`, `sendcan`, `carParams`, `carState`, `carControl`, `controlsState`, `selfdriveState`, `onroadEvents`, `pandaStates` and `deviceState`. A reduced CAN-only schema can omit other services. Logging-related hardware settings can also affect available evidence.

## Panda build connection

The repository SConstruct calls `panda/SConscript` and `panda/board/SConscript`. `panda/board/main.c` includes the safety root through the existing `opendbc` link to `opendbc_repo/opendbc`, then `safety/safety_hyundai.h`. `panda/firmware_version.py` hashes safety source content. `pandad.py` compares the selected image's signature, while `panda_safety.cc` applies CP.safetyConfigs.

After an authorized installation, confirm both the built H7 signature and the actual device signature/configuration. This integration does not replace the Panda tree, bypass firmware verification or modify the pedal-device firmware.
