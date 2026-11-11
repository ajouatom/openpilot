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
All 600 pandaStates samples show faultStatus `faultTemp` and fault
`interruptRateCan2`. Here `faultTemp` means temporary fault status.

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

## Older-version normal-driving comparison, 2026-11-11

The owner supplied `00001f02--55bd6d59a1--0` and `--1` as normal-driving
examples after returning to commit `2ea4415ac24d3a76f48a66d2834382d5717384ee`.
The recordings themselves are from November 11 around 08:10–08:12 KST.
October 21 is the commit date. The earlier failure `00001ef4--23d781cad3--12`
was recorded November 9 around 08:09 KST at
`b1c827468fc44f7a77958c658159b651953b9c7d`.
Both recordings identify HardwareSerial `ac960474`, tici/DOS, AGNOS
`19.8-carrot-bt1`, HYUNDAI_CASPER_EV, CP flags 67976 and Hyundai safety param 13.
Both use EnableRadarTracks=0, EnableCornerRadar=0, HyundaiCameraSCC=1 and active
openpilot longitudinal control. Recorded stopping, actuator-delay and following-gap
parameters checked in this comparison also agree.

| Measurement | Older version, segment 0 | Older version, segment 1 | Failure, segment 12 |
| --- | --- | --- | --- |
| Raw received SCC11 packets | 3,163 | 2,997 | 2,990 |
| Plausible status1/valid1 objects below 150 m | 3,056 | 2,997 | 2,217 |
| Of those, fixed 10.4 m / +0.2 m/s | 0 | 0 | 2,163 |
| SCC11 counter discontinuities | 0 | 0 | 0 |
| Maximum SCC11 interarrival gap | 30.44 ms | 31.09 ms | 30.69 ms |
| SCC/vision/absent source changes | 5 | 0 | 176 |
| Raw FCA fault packets | 4 | 0 | 297 |
| canState1 error-passive samples | 632/632 | 600/600 | 600/600 |
| canState1 totalErrorCnt growth | 245,809 | 244,634 | 606,231 |
| interruptRateCan2 fault samples | 0 | 0 | 600/600 |

Normal-driving segment 0 has four FCA fault packets near 13.11–13.17 s.
Segment 1 has continuous SCC selection, including 910 packets at 4.5 m and
zero relative speed. The owner's normal-driving description is consistent with
these varying SCC measurements and stable selection. Physical braking performance
was not independently measured in this comparison.

### What the normal logs change in the diagnosis

The separate canState1 error-passive condition exists during both normal-driving
segments. Its error IRQ/reset growth in segment 1 is about 4,084/s, compared
with about 10,121/s in the failure. The F4 CAN interrupt-rate threshold is 12,000/s
and the failure has individual observed intervals above that threshold. The
normal example therefore prevents treating error-passive alone as an explanation
for the SCC object freeze. The earlier severe error storm remains real, and its
causal relationship to SCC perception remains unresolved.

All 9,150 SCC11 packets match independent little-endian field extraction and
the repository CANParser. The fixed object is present in received bus-2 bytes.
The recorded failure chain is received fixed/invalid SCC content, acceptance of
the repeated valid-looking object, SCC/vision handoffs, and changing controller
inputs. The earlier fault-admission correction addresses continuing control under
the observed faults. It does not identify the trigger of the received object freeze.

### Version comparison and recorded-input replay

Between the two recorded commits, these sources are byte-identical

- `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- `opendbc_repo/opendbc/dbc/hyundai_kia_generic.dbc`
- `opendbc_repo/opendbc/car/hyundai/hyundaican.py`
- Panda board drivers and `openpilot/selfdrive/pandad`

The CameraSCC MAIN-availability change was reviewed against the actual data.
Both segment 1 and the failure have cruiseState.available=true throughout,
outgoing SCC11 MainMode_ACC=1 throughout, received MainMode_ACC=0 throughout
and received SCC12 ACCMode=0 throughout. This change does not explain the
difference in these recorded windows.

Added CommaPedal paths require another vehicle profile and flags. The recorded
Casper flags and safety param do not enable those paths. The recorded outgoing
CAN address set and periodic counts agree between segment 1 and the failure.
Neither window contains a direct sendcan command to bus 2 or diagnostic traffic.
For vehicle-side receive packets forwarded to camera-side TX echo, every one of
115,387 packets in normal segment 1 has an identical payload within the
comparison window. In the failure, 115,479 of 115,480 match. The single unmatched
packet is address 0x490. There is no broad forwarding loss or modified-payload
pattern in these windows. Startup is excluded from this steady-state comparison.

The private replay loads each commit's complete Python `radar_motion` package in
isolation, preserving source hashes. It runs both implementations on identical
recorded model, liveTracks, carState and pose inputs with EnableRadarTracks=0.
At each model publication it uses the latest preceding recorded inputs and the
production exposure/publication time calculation. It also executes each commit's
actual `_update_scc` method on every received SCC11 packet with recorded ego speed.

| Replay comparison | Segment 0 | Segment 1 | Failure |
| --- | --- | --- | --- |
| SCC method calls | 3,163 | 2,997 | 2,990 |
| Different SCC method outputs | 0 | 0 | 0 |
| Complete radar-controller calls | 1,152 | 1,199 | 1,200 |
| Different radar-controller outputs | 0 | 0 | 0 |
| Fixed object accepted by either SCC method | 0 | 0 | 2,083 |
| Fixed object selected by either radar controller | 0 | 0 | 787 |

Thus the older source also accepts the failure's input. These observations provide
no reproduction of a version regression in the decoded SCC/lead-selection path.
They establish an existing weakness in trusting plausible repeated SCC content.
The replay uses Python backends and a common model-publication schedule. It does
not reconstruct original process scheduling or execute the vehicle ECU/Panda binary.

### Remaining limits and evidence needed for the upstream cause

The normal logs record dirty=true with an empty saved GitDiff. Their exact local
source changes cannot be recovered from these logs. The failure records dirty=false.
The saved PandaSignatures parameter also differs between versions. This is a
recorded firmware-signature parameter, and does not independently verify the
flashed binary. The good recording starts near Panda uptime 5 s and continues
through the first two minutes. The failure segment starts near uptime 729 s.
Version, restart and accumulated runtime are therefore confounded.

The failure route's NAS directories for segments 11–14 are now empty. The archived
segment 12 remains available. No surviving startup/first-fault transition from
that route was located in this task. The two-version source replay cannot establish
whether the upstream trigger was ECU state, an electrical/runtime condition, an
initialization interaction or another executable difference. A same-condition
capture beginning before the first SCC/FCA transition would distinguish those
possibilities. A specific causal commit has not been established.

Analysis made no production control or radar changes. Private inputs, decoded
evidence, comparison scripts, source snapshots and replay results are archived at
`.analysis/archive/2026-11-11/casper-version/evidence.zip`. Rlog SHA256 values are

- Segment 0 `588dbc23e7c7450da4f05fe15215582557e0b0ade41ea67249a517eb3bd0a98f`
- Segment 1 `fc0384394ac52e106baa74225cea4792009d8685ed70ddf91892e4b82941060b`
- Failure `34f00df4794598576bd59e5135c62501941109f4ae20da95e19521197175e781`

### Owner's stock-control and intermittent-onset follow-up

The owner additionally reports normal operation with stock SCC, and intermittent
failures with recent software both immediately after starting and later during
driving. These are owner-reported observations. The stock test's comma/harness
connection state and the recent failing executable revision are awaiting clarification.
Together with normal older-version operation, this increases the priority of
software-dependent ECU interaction and startup/state transitions in the investigation.
The received-byte evidence identifies where the abnormal content is observed.
The cause can still involve software that influences the transmitting ECU.

An additional offline check executed the actual `CarInterface.init` methods from
2ea4415ac, b1c827468f and ecdd043934 with recorded Casper CP flags 67976,
extFlags 20 and EnableRadarTracks=0. All three make only the same
LongitudinalPersonalityMax=4 parameter write. None attempts ECU-disable, radar
track enablement or blinker-ECU diagnostics in that configuration. Recorded
CAMERA_SCC also excludes the periodic tester-present branch in all three.
Params writes and CAN functions were captured by offline spies, with no device IO.
This narrows those specific diagnostic paths. The earlier received-data replay
and this initialization check leave full runtime/ECU interaction unresolved.

The owner then clarifies that operation is normal when SCC data is used directly
without blocking. This investigation interprets that description as forwarding
stock SCC commands to the vehicle. The exact configuration or implementation of
that successful path has not been supplied. This observation strengthens the
priority of investigating command substitution and its ECU feedback effects.

The actual `hyundai_fwd_hook` is byte-identical across 2ea4415ac, b1c827468f
and the November 11 source. Accepted host SCC12 transmissions suppress forwarding
of stock SCC11/SCC12/SCC14 for 400 ms. FCA11 has its own 400 ms suppression timer.
The recorded host SCC12 maximum transmission gaps are 23.97, 23.78 and 23.64 ms
in good segment 0, good segment 1 and the failure respectively. Normal older-version
operation therefore already includes this substitution mechanism.

After excluding the first 0.5 s following each segment's first host SCC12, original
SCC11/SCC14 payloads have no matching vehicle-side TX echoes in any of the three
logs. The sole failure SCC12 payload overlap also matches a preceding host send,
so it does not establish stock passthrough. Stock SCC13 and FCA12 payloads continue
through in all three windows, with no host replacements for those addresses.
FCA11 echo identity alone cannot distinguish forwarding from a host copy with
unchanged fields. This evidence narrows intermittent forwarding takeover as an
explanation for the supplied windows. Timing or ECU interactions outside those
windows remain unresolved. Private results are in `forwarding-followup.json`.
