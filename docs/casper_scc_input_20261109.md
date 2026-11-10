# Casper EV SCC input and CAN-health investigation (2026-11-09)

## Findings

The newly supplied local `T:/00001ef4--23d781cad3--12--rlog.zst` is a
60-second Casper EV/tici log, not the Sonata from the earlier conversation.
It records clean carrot-wip commit `b1c827468fc44f7a77958c658159b651953b9c7d`.
The owner confirms a stock vehicle configuration without an additional CAN
signal modification device. No vehicle connection or modification was made.

Two distinct abnormalities are established:

1. Received camera-side SCC11 repeatedly reports a fixed 10.4 m / +0.2 m/s
   lead and alternates with invalid object data. Mode 0 accepts that SCC lead,
   causing frequent changes between SCC and vision-derived leadOne.
2. Panda hardware CAN2 (`canState1`, normally logical bus 1) continuously
   reports receive errors and excessive error interrupts. Its error counter
   grows by 606,231 during 59.900 seconds. This is an active CAN-health problem,
   not merely a historical fault or a user-facing timeout count.

The SCC reception channel is logical bus 2, a different channel from the
erroring CAN2 controller. SCC11 packets arrive continuously with correct counter
progression. Neither fact cancels the other: packet continuity does not establish
healthy connectivity everywhere, and CAN2 errors do not prove that they cause
the SCC object's fixed values. A specific sensor, connector, bitrate, transceiver
or ECU failure has not been identified.

## Actual input path

Recorded settings: `EnableRadarTracks=0`, `EnableCornerRadar=0`,
`HyundaiCameraSCC=1`. CarParams identifies `HYUNDAI_CASPER_EV`, flags 67976,
classic Hyundai safety, longitudinal control enabled, radarTimeStep 0.05 s.

The exact recorded source selects:

1. Physical receive `can`, bus 2, address 0x420 / SCC11, using
   `hyundai_kia_generic.dbc`.
2. `RadarInterface._update_scc`: ACC_ObjDist -> dRel, ACC_ObjRelSpd -> vRel,
   negative ACC_ObjLatPos -> yRel. It rejects non-status objects, distances
   >=150 m and one-cycle changes exceeding 3 m or 1 m/s. The SCC identity is
   track 0, with source `scc`.
3. `liveTracks` publishes measured SCC points, otherwise an empty list here.
4. `DPathRadarController` mode 0 uses `unconditional_scc_match`; it does not
   require agreement with the model's lead range. If there is no eligible SCC
   point, the recorded lead falls back to vision.

This is the OEM-selected object stream, not a raw multi-object front-radar list.
The log does not establish which physical perception component generated that
object upstream of the camera-side CAN interface.

Relevant source locations: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
(`get_radar_can_parser_scc`, `_update_scc`),
`openpilot/selfdrive/carrot/radar_motion/controller.py` (mode 0 admission), and
`opendbc_repo/opendbc/dbc/hyundai_kia_generic.dbc` (SCC11).
Those relevant files and the parser/DBC implementation were unchanged between
the recorded commit and the inspected checkout.

## Received SCC evidence

- 2,990 SCC11 bus-2 RX messages over approximately 60 seconds; median interval
  20.183 ms, maximum 30.691 ms. Every one of the 2,989 counter transitions
  advances by one modulo 16.
- 2,217 messages report status=1, ObjValid=1 and a plausible range. Of those,
  **2,163 (97.56%)** report exactly 10.4 m, +0.2 m/s and 0.0 m lateral position.
  A representative raw payload is `20ff41c8d0606a00`; counter changes continue
  while the object fields remain fixed. The few other plausible values mostly
  occur briefly around 26–34 seconds.
- 514 messages report 204.6 m / 239.4 m/s, status=0 / valid=0. Another 259
  report the all-ones field maxima 204.7 / 239.5 / 31.1, status=2. These values
  are rejected as objects; they are not actual measured long-range targets.
- There are 80 raw invalid-object runs, approximately 0.060–0.321 s long
  (median 0.200 s). Additional admission gaps follow the existing jump filter.
- All 1,200 liveTracks messages are valid as services: 796 contain an SCC
  point, 404 are empty. Of the 796 points, 786 retain 10.4 m / +0.2 m/s.
- All 1,200 radarState leadOne entries have status=true: 796 select SCC
  track 0, 404 use vision track -1. Source changes occur **176 times**, with
  median adjacent-frame range change 33.01 m and maximum 107.84 m.

For example, leadOne is SCC 10.39 m near 0.013 s, vision 41.09 m near
0.364 s, then SCC 10.39 m near 0.612 s. Ego speed changes from about
20.9 to 64.3 km/h within the segment; model lead x spans approximately
14.3–119.7 m. These model estimates are not independent ground truth, but
they establish that the planner is receiving radically different object inputs.
No video-based physical range verification was performed in this task.

The actual repository CANParser reproduced the independent little-endian
bit-field decode for all 2,990 SCC11 messages. Params access was blocked in
that offline decoder. RX bus 2 is separate from sendcan bus 0 / TX echo bus
128. None of the complete bus-2 SCC11 payloads equals a complete transmitted
bus-0 SCC11 payload in the segment. This rules out directly decoding the
logged TX echo as the input, but not an unobserved ECU interaction.

## FCA state accompanying object loss

Raw bus-2 FCA11 contains 297/2,990 messages where `FCA_Status=3` or
`FCA_Failinfo!=0`; 261 have failinfo=3 and 259 have status=3, with overlap.
No CF_VSM_DecCmdAct is asserted. Near the first SCC loss at 0.337 s,
FCA_Failinfo changes to 3, then FCA_Status becomes 3. Object data returns
at 0.517 s after the fault indication clears.

The recorded Casper-specific `suppress_casper_ev_fca11_fault` clears
FCA_Failinfo and changes faulted FCA_Status to 2 when forwarding FCA11.
All 3,000 outgoing FCA11 messages consequently show status=2, failinfo=0.
This means the outgoing view cannot establish that the received OEM state is
healthy. The investigation does not remove or otherwise modify this treatment.
The fault-field values are not a diagnostic trouble code identifying a component.

## Connection-health follow-up

The owner explicitly asked whether unstable connectivity could be responsible.
All 600 pandaStates samples show `faultTemp` and `interruptRateCan2`.

| Counter / state | canState1, first -> last |
| --- | --- |
| totalErrorCnt | 5,636,727 -> 6,242,958 (+606,231) |
| canCoreResetCnt | 5,636,728 -> 6,242,959 (+606,231) |
| receiveErrorCnt | 255 throughout |
| errorWarning / errorPassive | true throughout |
| lastError | stuffError / formError / crcError |
| totalRxCnt / totalTxCnt | zero throughout |
| error IRQ rate | 7,180–14,232 calls/s |

In the recorded F4 bxCAN implementation, this cumulative error counter increments
on error IRQ handling; the reset counter increments on `can_clear_send` during
that handling. These are not 606,231 lost SCC packets, vehicle ECU resets or
independent physical disconnects. `interruptRateCan2` names hardware CAN2,
which corresponds to canState1; it must not be confused with logical SCC bus 2.
Harness flipping swaps bus 0/2 and does not swap this middle controller.

canState0 and canState2 have zero total-error growth, zero receive/transmit
error counts, no bus-off and no error-passive state. Their totalTxLostCnt grows
796/2,477; bxCAN counts arbitration-lost/transmit-error attempts here, so these
increments alone do not establish permanently dropped frames. There is no
RX FIFO loss or top-level RX/TX buffer overflow. Safety RX invalid, safety TX
blocked, heartbeat loss and SPI checksum error counters remain zero. All
6,000 carState samples are canValid with no timeout or accumulated CAN error.
Peripheral supply is 13.010–13.234 V; this does not rule out short transients.

Thus the strong statement supported by the log is **an actively unhealthy
separate CAN channel plus abnormal SCC object/FCA content**. The CAN2 physical
connection, configuration and electrical/transceiver condition need separating
from upstream SCC perception and handoff behavior. The available segment does
not establish that repairing one will cure the other. No live wiring experiment,
diagnostic ECU request, setting change, firmware change or controller patch was
performed.

## Reproduction

Full OpenPilot cereal decoding was used. The 11,485,755-byte source SHA256 is
`34f00df4794598576bd59e5135c62501941109f4ae20da95e19521197175e781`.
Times in this report are relative to the first carState, not initData.
The scripts, copied source log, raw CAN extraction, independent decode,
CANParser verification and Panda state evidence are private under
`.analysis/archive/2026-11-09/casper-scc-1ef4/` and excluded from publication.
Unrelated working-tree changes were preserved.

## Implemented fault admission follow-up

The recorded event checks only admitted relay failure, safety mismatches and
message validity/timeout failures; they did not consume CAN interrupt faults or
current per-controller error growth. Replaying the exact original Panda/CAN
checks on all 600 recorded Panda samples produced zero events. Relay and safety
RX-invalid positive controls still triggered their original events.

The correction monitors fresh Panda publications, confirms at least 0.5 s of
current error-passive/bus-off state with the matching CAN interrupt-rate fault
and error growth of at least 100/s, then emits a dedicated immediate-disable,
no-entry and persistent alert. Old counters or a stored fault bit alone do not
qualify. Duplicate samples, gaps, invalid input, counter reset and uptime changes
cannot complete confirmation or clear an established fault. Recovery requires
two continuous seconds of fresh non-passive/non-bus-off observations without
qualifying error growth.

Casper's original FCA fault is additionally published as `stockSccFault` before
the existing outgoing sanitization. At least 0.10 s of fault samples within one
second confirms a recurring fault; two healthy seconds are required to clear it.
Both confirmed faults block control and re-engagement. Controlsd independently
applies the fault events to enabled/latActive/longActive so AlwaysLateral cannot
keep steering after this failure. This does not modify the original CAN input,
remove the outgoing FCA compatibility treatment, or claim a hardware repair.

Recorded-input replay first confirms the CAN fault 0.591 s into the segment and
the recurring raw FCA fault about 2.141 s in. The latter reconstructs fault state
from the original received FCA messages; it is not a log from modified software.
Dedicated event/state-machine and full state_control tests check the inhibit.
The new CAN-FD stop-object trial is separate from this classic-CAN Casper case;
see `docs/canfd_signal_stop_object_20261109.md`.
