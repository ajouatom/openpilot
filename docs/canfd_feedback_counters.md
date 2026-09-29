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
