# CAN-FD camera feedback counters

## Change (2026-09-27)

Camera-SCC MDPS and TCS feedback now remove `COUNTER` from the copied input
values and pass it as `rx_counter` to the existing CANPacker. The first emitted
counter is the received counter plus one; subsequent transmissions advance the
per-message counter modulo 256, independently of repeated or skipped input
counter values. Missing snapshots produce no message and do not advance it.
The packer recalculates the checksum after assigning the outgoing counter.

This preserves the existing transmission cadence, buses, feedback field
modifications, and original receive snapshots. It does not implement a queue
that forwards every received frame exactly once: feedback still uses the latest
snapshot at each scheduled transmission. MDPS and TCS have separate sequences.
Button-message forwarding is outside this change.

## Evidence and limits

A 60-second GV70 capture on c04fa456 had consecutive original MDPS counters,
but camera-bound MDPS contained 254 repeated counters and 253 increments of two.
TCS counters were consecutive in that capture; its identical explicit-counter
copying pattern is corrected as well. A camera-side warning and LFA state change
were recorded, but causality between the warning and counter behavior is not
established. This change is not a demonstrated vehicle warning fix.

`test_canfd_feedback_counters.py` checks repeated/skipped input counters,
wraparound, absent snapshots, independent MDPS/TCS sequences, source immutability,
unchanged payload behavior, and CRC with an independent CRC implementation.
All three focused tests passed on Windows with only the unavailable native Params
module stubbed. The combined packer/parser run passed 19 tests and failed the
existing `test_parser_can_valid` initial-validity assertion; that failure also
reproduced using the unchanged HEAD Hyundai message implementation.
Vehicle acceptance and warning recurrence require on-device validation.

## Follow-up warning recurrence (2026-09-29)

GV70 route `000002f2--858b6e2e0b--26`, reported revision `962d4484`,
contains the counter fix `1ab3448148`. Full cereal decoding and the incident
revision DBC show 5,999 original MDPS frames and 6,009 camera-bound MDPS
transmissions; every counter step is +1. TCS RX/TX steps are also consecutive.
Nevertheless the previous camera-warning/cluster-popup pattern recurs. Thus
counter continuity alone did not eliminate this observed warning pattern.
The capture reports dirty source, so commit identity is not a byte-for-byte
attestation of the device checkout.

Times below are relative to the first carState message, not initData:

| Signal | Previous capture | Current capture |
| --- | --- | --- |
| Brake/selfdrive disengagement | +4.601 s | +26.394 s |
| Camera STEER_REQ briefly clears | +10.526 s | +34.096 s |
| Camera STEER_REQ returns | +10.607 s | +34.166 s |
| Camera FCA_SYSWARN rises / LKA_ACTIVE clears | +10.677 s | +34.217 s |
| Camera LKA_MODE changes 2 to 7 | +10.687 s | +34.227 s |
| Camera HDA_InfoPUDis becomes 3 | +10.677 s | +34.227 s |
| Vehicle-bound HDA_InfoPUDis becomes 3 | +10.726 s | +34.261 s |

Current FCA_SYSWARN lasts about 2.01 seconds; the outgoing popup request is
present for 0.10 seconds. The latter is not a measurement of dashboard display
duration. DBC value 3 is "system auto disengaged pop-up", distinct from value 4
(system Fail). VALUE63 changes 0 to 15 with the warning in both captures; its
meaning, and the meaning of LKA_MODE 7, are not independently established.
Actual dashboard wording and an MDPS ECU DTC cannot be recovered from these
signals alone.

### Ranked investigation candidates, not established ECU fault logic

1. **Camera-side steering feedback consistency.** Original MDPS LKA_ACTIVE
   remains 1 while vehicle-bound LFA STEER_REQ remains 1, including after
   longitudinal/selfdrive disengagement. However camera-bound MDPS LKA_ACTIVE
   is overwritten with the camera's own STEER_REQ. The camera therefore sees
   a synthetic inactive/active acknowledgement roughly 4-5 ms after its request
   changes, while actual EPS output torque and other MDPS fields remain copied
   from the independently controlled vehicle. Both warnings follow a short
   camera request-off/re-enable cycle by 50-70 ms. This is a concrete feedback
   inconsistency candidate, not evidence that forwarding the original active
   bit would be correct. Four other 80-90 ms request-off cycles across the two
   segments have no immediate warning, so this sequence is not sufficient.
2. **Camera-side brake/ACC state consistency.** TCS forwarding forces
   DriverBraking to 0 and changes NEW_SIGNAL_1 to 1 when ACC_REQ becomes 0.
   The camera's SCC ACCMode remains 1 while vehicle-bound ACCMode becomes 0.
   Both incidents occur after brake disengagement, but their delays differ
   (about 6.1 versus 7.8 seconds). A second brake press occurs 111 ms before
   the current warning; no equivalent immediate press occurs in the previous
   incident. Brake-edge-only causality and a fixed timeout are unsupported.
3. **Button counter forwarding remains a separate protocol candidate.**
   Current input has 200 +2 steps; output has 111 repeats and 18 +3 steps.
   Input is already discontinuous. No nonzero driver cruise/LFA button occurs;
   the injected LFA button is +36.032 s, after warning onset, so it cannot
   initiate this warning. ECU acceptance of the counter pattern is unknown.

Current artificial column-torque +220 windows are +26.012..26.414 and
+36.012..36.414 s around the warning at +34.217 s; previous corresponding
windows were +3.377..3.771 and +13.372..13.771 around +10.677 s. No pulse overlaps
warning onset. This weakens an immediate pulse-trigger hypothesis without
excluding a delayed effect. Original LKA_FAULT/LFA2_FAULT and SCC SysFailState
remain zero; full-log state checks show no CAN fault or model/pose invalidity.
UI FPS slowdown is a separate observation, not an established popup cause.

The strongest next discriminator is synchronized ECU diagnostic information
and a controlled parked comparison of camera feedback consistency, where
reproduction is possible. Do not suppress popup/fault fields or change steering
feedback semantics solely on this correlation. Existing captures identify the
camera state transition and remaining inconsistent feedback, but not the
camera's internal reason for rejecting/disengaging. No runtime change was made.
Private scripts and evidence: `.analysis/archive/2026-09-29/gv70-cause/`.

## Matched conditions and negative controls (2026-09-29)

The two captures have the same observed warning signature, not merely the same
cluster appearance: FCA_SYSWARN 0->1, VALUE63 0->15, LKA_ACTIVE 3->0,
LKA_MODE 2->7->2, and HDA_InfoPUDis=3. This supports treating them as recurrence
of the same camera-side event family, without establishing identical ECU DTCs.

Clarification: selfdrive/longitudinal disengagement is **not lateral shutdown**.
At all six short camera request-off/re-enable cycles below, carControl has
`enabled=false, latActive=true, longActive=false`, original TCS ACC_REQ=0,
and vehicle-bound LFA STEER_REQ=1. Camera-bound MDPS mirrors the camera request
while actual MDPS remains active. This mixed state also occurs without warning.

| Capture / re-enable time | Warning within 200 ms | Speed km/h | Brake pressed | Steering torque, decoded DBC units | MDPS active feedback delay ms |
| --- | --- | --- | --- | --- | --- |
| Previous +7.498 s | No | 47.1 | Yes | -241 | 4.42 |
| Previous +9.487 s | No | 38.0 | No | -188 | 5.07 |
| Previous +10.607 s | Yes | 36.7 | No | -206 | 4.25 |
| Previous +15.287 s | No | 35.5 | No | -142 | 4.91 |
| Current +34.166 s | Yes | 71.5 | Yes | -389 | 5.98 |
| Current +59.807 s | No | 47.2 | No | -262 | 5.76 |

Values are sampled at camera request re-enable, not at warning onset. Torque
is not labelled N m: these DBC column-torque units must not be assumed physical
units. All six have both blinkers off. Braking at that instant, a single speed
threshold, torque magnitude, camera icon state, and MDPS feedback latency do
not separate the two warnings from all four non-warning cases. This does not
rule out history-dependent or combined conditions.

For the interval from 200 ms before request-off to 50 ms after re-enable, both
warning cases have camera-bound button counters with fifteen +1 steps and one
+2 step (current has fourteen +1 and one +2); input has the same patterns in
those cases. There are no extra outgoing repeats or +3 steps in these windows.
Current warning-window MDPS steps are all +1, with max host-publication gap
13.13 ms; its non-warning comparison is 12.43 ms. Prior warning-window max is
14.56 ms versus up to 14.94 ms in a non-warning window. No exceptional transport
stall or button-counter anomaly distinguishes the warning onset here.

Additional same-revision GV70 segment `000002f0--aa7506f9ed--3` has no camera
warning, popup or short request-off/re-enable cycle. Its speed is 0..18.7 km/h,
with 24.98 seconds of lateral-only control and the rest fully inactive. It is
a low-speed non-warning comparison, not a matched validation of the suspected
higher-speed transition. Do not infer a speed threshold from it.

Next comparison should preserve separate clocks and states for actual EPS,
camera command, modified EPS feedback, brake/ACC, and camera warning, and
collect diagnostic reason information around a naturally recurring event.
These offline observations do not justify changing the active-bit semantics,
forcing the camera enabled, or suppressing the warning. Any target-side A/B
needs a concrete test scope and independent assessment of steering effects;
no such deployment or runtime modification was performed in this analysis.
Private reproduction: `.analysis/archive/2026-09-29/gv70-conditions/`.

## Authorized popup suppression and sound check (2026-09-29)

The user explicitly requested blocking the recurring popup despite the unknown
root cause, then requested checking the sound as well. This supersedes the
prior recommendation above to postpone suppression. This is display handling,
not a fix for the stock camera's internal warning or a diagnosis of its cause.

The controller enables suppression only for `GENESIS_GV70_1ST_GEN` with
camera-SCC. In `create_lfahda_cluster`, it applies only when all conditions hold:

- lateral active and longitudinal inactive;
- HDA_InfoPUDis=3, HDA_InfoPUDis1=0, HDA_LFA_WrnSnd=0;
- camera LFA FCA_SYSWARN=1 and VALUE63=15;
- MDPS LKA_FAULT=0 and LFA2_FAULT=0, SCC SysFailState=0.

Only outgoing HDA_InfoPUDis changes to 0. Original snapshots/raw CAN remain
unchanged. Other popup values, separate sound requests, decoded faults, missing
evidence, other control states and other platforms retain existing behavior.
These conditions match both captures but do not establish a unique ECU cause;
a future event with the same signature will also be suppressed.

Both complete captures have HDA_LFA_WrnSnd=0 and HDA_InfoPUDis1=0 on camera RX
and vehicle-bound TX. At warning onset, selfdriveState has alertSound=none and
no openpilot alert. The reported sound could be generated by the cluster in
response to the popup request; no independent sound command was identified in
this message. Suppressing the popup request should remove that trigger, but
neither this inference nor actual audible silence is validated by CAN replay.
No global audio mute or independent warning-sound suppression was added.

Validation:

- 37 focused popup and feedback-counter tests pass on Windows, stubbing only
  the unavailable native Params module. Coverage includes popup values 0..7,
  other popup/sound requests, MDPS/SCC fault coexistence, missing evidence,
  control states, source immutability, counter wrap and independent CRC.
- Offline replay using received snapshots at logged cluster TX times checks
  2,401 outputs across the two captures. All four observed popup frames are
  suppressed: current +34.261/+34.311 s; previous +10.726/+10.772 s. Only the
  popup bits and checksum differ from the same packer path without suppression.
  Initial sends without an available RX snapshot are excluded. This is output
  reconstruction, not execution of the entire vehicle controller or ECU replay.
- Focused Ruff and `git diff --check` pass. No steering, radar selection,
  MDPS/TCS feedback or control scheduling changes are included.

Physical GV70 display and sound behavior remain to be verified after updating.
Private reproduction: `.analysis/archive/2026-09-29/gv70-popup/`.

## Include active longitudinal control (2026-10-09)

The authorized GV70 CAMERA_SCC popup filter now also applies while longitudinal
control is active. The controller's existing `GENESIS_GV70_1ST_GEN` and
CAMERA_SCC restriction is unchanged; lateral control must still be active.
Only the `not long_active` condition is removed. The exact popup/signature,
zero separate sound request, and absence of decoded MDPS/SCC faults remain
required. Other vehicles, popup identities, missing evidence and lateral-off
states retain their previous behavior. This affects the display request only,
not the original camera warning, control commands or CAN scheduling.

Validation: 211 popup, feedback-counter, DM-cluster and direct-TX configuration
tests pass, including longitudinal ON/OFF fault and sound preservation. Two
new longitudinal-ON cases fail against the previous implementation. A local
recorded-snapshot replay of 1,199 cluster outputs changes only the two matching
popup requests and their CRCs; counters and other bytes are unchanged. The
underlying ADAS warning cause and physical popup/chime suppression remain
unverified. This is a host change; no new Panda firmware change is required by
this extension. Incident captures and identifiers remain local only.

## Sorento HEV scope extension (2026-10-09)

The user authorized the same observed-popup filter for
`KIA_SORENTO_HEV_4TH_GEN` with CAMERA_SCC. Two captures show the same camera
FCA_SYSWARN=1 / VALUE63=15 signature and HDA_InfoPUDis=3 request, without decoded
MDPS/SCC faults or a separate cluster sound request. The controller now enables
the existing predicate for this exact platform as well as GV70. Other Sorento
variants and other platforms remain excluded. Lateral control must still be
active; longitudinal ON/OFF, fault guards, other popup identities and raw camera
evidence keep their existing behavior. No steering, forwarding, timing or Panda
firmware change is included.

Recorded-snapshot comparison covers 2,399 cluster outputs and changes only the
three matching popup frames and their CRCs, retaining counters and other bytes.
Neither capture contains recorded audio, so a popup-associated chime remains an
inference. The shared underlying camera-warning cause and physical display/sound
suppression remain unvalidated. Incident files and reproduction evidence stay
local. Controller scope tests exercise the actual cluster output for both target
platforms, three excluded platforms, CAMERA_SCC ON/OFF, and lateral/longitudinal
ON/OFF. The two eligible Sorento cases fail before this change; all 251 focused
popup, feedback-counter, DM-cluster and direct-TX tests pass afterward on desktop
with native Windows Params storage substituted.
