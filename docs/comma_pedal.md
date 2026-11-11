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

## Ray EV recorded-response investigation, 2026-11-11

The owner asks about poor acceleration/deceleration on device 53e037b67afdc67d,
serial 159e3a01. This case uses the Ray Comma Pedal path. The previously discussed
fixed SCC11 object input belongs to Casper ac960474 and remains a separate case.

The supplied route `00000002--fbec2ccc7e--0` has 5,927 carControl publications,
all with longActive=false and actuator accel=0. All 1,385 pedal commands have
ENABLE=0. The recording covers manual driving after startup, so it cannot establish
active pedal-controller response. Full cereal decoding and raw command bytes agree.

The same device also has `00000003--90b0726b2d--0` on the NAS. It was inspected
as an additional available sample. This is the source of the active-control
measurements below, and the owner's intended incident correspondence remains
unconfirmed. Both logs record `df90d30833d5f0a180aaec61db94a3b4dd0047e7`,
dirty=true. The supplied upload header reports ecdd0439. The recorded executable
identity takes precedence for this analysis, while dirty state prevents exact
installed-source reconstruction. The first recorded clock times are approximately
11:36:48 and 12:04:48 KST on November 11, respectively.

### Active-control evidence

The additional route has two longActive intervals, approximately 37.944–38.753 s
and 43.163 s through the end at 59.463 s. Driver accelerator override occupies
the interval between them. Times are relative to the first carState.

| Measurement during 46–55 s | Recorded value |
| --- | --- |
| Requested acceleration | −0.245 to −0.113 m/s², mean −0.179 |
| Negative requests | All 900 carControl samples |
| Actual acceleration | Mean +0.146 m/s² |
| Vehicle speed | 70.88 → 75.61 km/h |
| Internal normalized pedal command | 0.418–0.505, mean 0.464 |
| Vehicle-reported accelerator input | 34.3–41.3%, mean 38.2% |
| Driver gas/brake pressed | Zero samples for both |
| Selected front lead | Zero present samples |

The requested deceleration and measured response disagree over nine seconds,
well beyond the short transition at engagement. The pedal command and the
vehicle-reported accelerator input use different scales. The latter is the
vehicle's reported input after the interceptor, while driver override is derived
from the separate pedal sensor stream.

All 1,387 sent commands and 3,025 received sensor packets in the additional route
pass the protocol CRC check. Command and sensor counters are continuous.
During the 37.94–59.46 s inspection window, pedal STATE remains zero, Panda faults
are empty, safetyTxBlocked remains zero and all three CAN total-error counters
have zero growth. Old cumulative fault counters precede this window. These
measurements prioritize the command-to-response control path in this sample.

### Two concrete control concerns

The actual Ray mapping gives normalized target 0.455 for a request of −0.2 m/s²
at 20 m/s, with a sufficiently higher speed target. It computes speed offset
0.565 plus −0.2 × 0.55. The current command DBC encodes this as raw tracks
609/1196. The pre-expansion DBC would encode the same normalized target as
437/846. PR 570 expanded the raw command increment while retaining the target
offset/gain table. The recorded positive response to negative requests strongly
suggests that the resulting map does not deliver the requested acceleration in
these conditions. Road grade, regenerative-braking mode, interceptor firmware
and voltage calibration have not been isolated, so reversing the expansion is
not established as a fix by this observation alone.

The upstream Hyundai controller still uses speed-error P=1, I=0 and acceleration
feedforward F=1. Logged P/F terms are active and I is zero. The pedal adapter
uses the fixed speed/acceleration map without a measured-acceleration correction
or a learned steady-state offset. Persistent map error therefore has limited
compensation through the existing speed P term. The code has an operating
feedback path, and the recorded failure should not be described as missing PID
execution or F=0.

At both transfers from driver pedal use to automatic control, `gas_last` starts
from zero. The first commands are approximately 0.02, 0.04, 0.06, 0.08 and increase
by 0.02 every 40 ms. Executing the actual PedalInterceptor update method reproduces
the first eight raw command-track pairs at both transitions. Vehicle speed falls
59.74 → 57.71 km/h in the first transition, with minimum acceleration −1.095 m/s².
The second falls 69.55 → 67.90 km/h, with minimum −0.916 m/s², before acceleration
recovers. This supports inadequate pedal-output continuity at transfer as another
contributor to the reported uncomfortable response.

The speed taper also receives `hudControl.setSpeed`. In cruise xState=3 this is
derived from the planner's terminal speed and cluster ratio, so its changes must
not be interpreted as driver changes to the selected cruise setting. The observed
handover drop and sustained request/response mismatch are the primary findings.

### Scope and evidence

The priority for a corrective design is a Ray-specific calibrated pedal map,
appropriate feedback for sustained acceleration error, and a verified transfer
from driver input to automatic output. No numerical replacement calibration or
physical cure has been established by this two-minute sample. Production behavior
was not changed during this investigation.

Private full logs, decoded services/CAN, CRC checks, transition replay and chart
are archived under `.analysis/archive/2026-11-11/ray-pedal/`. Source hashes are

- Supplied route `695770387a03cc301b4006056700dd051bbf86d224657996bd367f94cbf88418`
- Additional route `871f12776e4d0ed6be629ee05c17fe9862ffdcae66a047db5c992dabbcb228bc`

### Follow-up implementation and initial gain estimate

The owner requested restoring Long Kp/Ki/Kf for Ray Comma Pedal while retaining
the speed PID. The shared capability check requires the Ray EV fingerprint,
CC_ONLY_CAR and PEDAL_INTERCEPTOR flags, and openpilot longitudinal control.
The settings catalog and snapshot expose these three controls for that capability.
LongControl reads their saved values on construction and through its existing
approximately one-second refresh. The stored-to-coefficient multipliers are
0.01, 0.001 and 0.01 respectively. The Ki menu title now reflects 0.001.

The existing defaults are 100/0/100. Previously saved values become effective
when the updated controller starts. The owner-specific initial candidate is
100/50/100, corresponding to Kp=1, Ki=0.05 and Kf=1. Ki=100 is a second comparison
candidate if persistent error remains. This selects a modest integral correction
for the observed sustained error. The sample does not establish optimal gains.

The additional route's logged P and F terms reconstruct the exact original
speed error and acceleration feedforward because its Kp and Kf were both one.
Replaying those inputs and recorded activation through production LongControl
reproduces all 1,713 active baseline outputs within 2.99e-8 m/s². Holding the
recorded trajectory fixed gives the following comparison for 46–55 seconds.

| Stored Kp/Ki/Kf | Mean acceleration request, m/s² | Mean normalized pedal target |
| --- | --- | --- |
| 100/0/100 | −0.1793 | 0.4641 |
| 100/50/100 | −0.2013 | 0.4520 |
| 100/100/100 | −0.2233 | 0.4400 |
| 150/0/100 | −0.2868 | 0.4055 |

Pedal targets use the production map before output slew limiting. At 55 seconds,
the I term is approximately −0.0722 for Ki=50 and −0.1444 for Ki=100.
Changing Kp also increases immediate reactions to speed error, so the initial
candidate keeps Kp=100 and adds Ki=50. Vehicle response, future speed error,
stability and comfort require closed-loop validation. The replay holds measured
motion fixed and establishes only the controller-output difference.

Regression coverage exercises first-frame saved gains, the actual speed-error
update, live refresh, integral reset on disengagement, other vehicle configurations,
real serialized CarParams, and catalog/snapshot visibility and values. The related
Hyundai/Toyota, settings and pedal suites pass 312 tests locally. Private comparison
code and results are archived under `.analysis/archive/2026-11-11/ray-tuning/`.
