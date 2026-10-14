# Elantra 29b segment 22 steering warning

Analysis only, 2026-10-03. No production code or vehicle settings changed.

## Input and timing

- User-supplied file: `T:\0000029b--52f66a3abc--22--rlog.zst` (11,485,463 bytes).
- SHA256: `0a6852b8d16eca05dc5ec37aecafddd7a52d63570db0cdce12ef03eef73af985`.
- Full cereal schema: HYUNDAI_ELANTRA, tizi/C3X, torque steering,
  openpilot longitudinal enabled, hyundaiLegacy safety parameter 12.
- Recorded clean revision `861fdb7324db33ebc8ee15e62d7f1976ff584b2b`,
  `heatagain/bosch-radar-tracks`; this is not the current carrot-wip revision.
  The recorded Git object is unavailable locally, so current source is only
  a cross-reference; conclusions below rely on recorded states and raw CAN.
- Zero is the first carState at monotonic 1417.091966638 s. Coverage is
  approximately 60 s. Video alignment was not separately established.
- NAS directory listing was inaccessible; the supplied local full rlog was
  successfully decoded. Service counts are retained with the private evidence.

## Confirmed warning chain

| Segment time | Observation |
| --- | --- |
| 9.008233 s | Original bus0 MDPS12 (0x251) changes ToiFlt 0→1, FailStat 0→1, ToiActive 1→0. |
| 9.021218 s | carState.steerFaultTemporary becomes true. |
| 9.023908 s | selfdriveState enters softDisabling with steerTempUnavailable/softDisable, warningSoft, and Korean text “즉시 수동 조작하세요 / 조향제어 일시적 사용불가”. |
| 9.025946 s | carControl.latActive becomes false. |
| 9.032555 s | Host LKAS11 requests zero torque with ActToi=0; Panda TX echo contains this at 9.037260 s. |
| 9.076659 s | Original MDPS fault and FailStat return to zero. |
| 9.080883 s | carState.steerFaultTemporary clears. |
| 9.082786 s | selfdriveState returns to enabled; the warning remains displayed. |
| 9.085115 s | carControl.latActive returns true. |
| 9.092260 s | Host requests torque -2 with ActToi=1; corresponding echo at 9.097044 s. |
| 9.119013 s | MDPS reports ToiActive=1 again. |
| 11.025387 s | The displayed/audible warning state clears. |

There are seven consecutive original MDPS fault frames, approximately
68.4 ms from first fault to first clear. carState also has seven fault samples,
with 59.7 ms between its first fault and clear. The approximately two-second
UI warning duration is not two seconds of continuous MDPS fault. Longitudinal
engagement remains enabled through this incident; lateral actuation briefly
stops and resumes. Host/batched CAN timestamps are not exact wire timestamps.

The immediate cause of the warning is the vehicle MDPS torque-overlay fault
report, specifically ToiFlt. ToiUnavail remains zero. The existing classic
Hyundai carstate decoder maps either flag to steerFaultTemporary, consistent
with the recorded transition. This is not evidence that the MDPS hardware
itself is defective: its internal reason for rejecting actuation is unresolved.

## Checks on possible triggers

- At detection, speed is about 61.2 km/h, steering angle -5.0 degrees,
  driver-column torque 30 raw units, and steeringPressed=false.
- Throughout the preceding nine seconds, steering angle is only -5.1 to
  -2.0 degrees; steeringPressed is always false. Echoed torque requests range
  from -65 to +23 raw CAN units, and from -37 to -10 during the last second
  before 9.008 s. The request at the first fault batch is -9. These values
  do not show a large-angle maneuver or a large torque command at onset.
- All preceding echoed commands in this segment have ActToi=1 and ToiFlt=0.
  The host's actuation-off transition follows the MDPS fault report; it does
  not precede it. The log starts while steering is active, so total continuous
  actuation time before the segment is unknown.
- All 5,997 host LKAS11 and 5,994 bus0 TX-echo LKAS11 frames have the expected
  additive checksum and continuous modulo-16 counters within each stream.
  All 5,997 original MDPS12 frames have valid additive checksums and continuous
  modulo-256 counters. There is no counter discontinuity at onset.
- In the two seconds before onset, maximum host LKAS publication interval is
  20.519 ms and maximum TX-echo batch interval is 20.857 ms. Batching and host
  scheduling prevent treating these as precise wire intervals or proving that
  every internal EPS timing requirement was met.
- All 5,995 carState samples have valid CAN, no CAN timeout and no permanent
  steering fault. All 562 Panda snapshots have no safety TX block, safety RX
  invalidity, buffer overflow, heartbeat loss or Panda fault. Existing CAN
  error/reset and SPI-checksum counters remain unchanged throughout the segment.
- All 1,199 livePose messages are valid with inputsOK/sensorsOK/posenetOK true.
  UI FPS warnings occur separately; they are not the recorded warning trigger.
- Original camera bus2 LKAS11 has ToiFlt=1 from the segment boundary and
  ActToi=0 throughout. That bit alone is not a new fault at nine seconds;
  distinguish it from original bus0 MDPS12 and host/bus0 TX commands.

No single underlying mechanism (EPS internal condition, command compatibility,
or timing) can be selected from these checks. There is no basis here to change
steering limits or suppress the warning. Previous segments and MDPS diagnostic
trouble-code/freeze-frame information would help identify the internal trigger.

## Additional occurrences

Original MDPS ToiFlt also rises at 42.198294 s and 43.377109 s, clearing at
42.357360 s and 43.526767 s. These later cases occur during large rightward
steering (roughly -110 to -128 degrees) with steeringPressed=true and brief
host ActToi=0/ToiFlt=1 pulses. Conditions differ from the small-angle nine-second
incident, so they should not be assigned the same cause without further work.
They are recorded as temporary steering faults and lateral interruptions,
but selfdriveState is already disabled, so no matching soft-disable warning
appears then. AlwaysLateral=1 is recorded in startup Params.

## Follow-up: could stock bus2 LKAS reach the MDPS?

All 5,994 recorded bus0 LKAS TX echoes (can.src=128) match the host's full
eight-byte payload in exact order, host indices 2 through 5995 inclusive.
No echo matches only the contemporaneous stock bus2 payload, and none is
ambiguous between the two. Echo-minus-host timestamps range 2.754–16.081 ms.
The three host packets without echoes are at segment boundaries (the first
two host packets and the final packet), not at the incident.

All 5,997 stock bus2 LKAS frames have ActToi=0 and zero requested torque;
5,989 have ToiFlt=1 and eight have ToiFlt=0. At 9.008233 s, stock bus2 is
`3c0000342400ac18`, while the bus0 TX echo is `0c00f72bc4000a18`, exactly
matching the host packet at 9.001369 s. There is no recorded stock-LKAS
insertion or takeover at the warning, or elsewhere in this segment.

Current classic Hyundai safety source blocks bus2 LKAS while the last accepted
host LKAS is newer than 200 ms; stock forwarding resumes after that timeout.
The freshness update uses accepted LKAS messages, not their ActToi bit.
Therefore the two-frame fault-avoidance pulse does not itself enable stock
forwarding: LKAS packets continue to be sent with changed bits. The largest
host LKAS publication gap in this segment is 31.141 ms, and 20.932 ms in
the 8–9.2 s window. The recorded revision/firmware is unavailable locally,
so this source explanation is conditional; the full-payload echo comparison
is direct incident evidence.

Current FDCAN code returns an echo when submitting a frame to the hardware TX
FIFO, including forwarded frames. It is not an independent wire capture or
proof of MDPS reception. Other stock messages and copied ancillary LKAS fields
are not ruled out by an LKAS-only comparison. No evidence presently identifies
them as this warning's cause.

## Follow-up: K7 camera-side MDPS feedback compatibility

The earlier K7 PE investigation and fix `a6f79b65e4` concerned original active
MDPS feedback reaching a camera whose own steering request was inactive. In
those logs the camera then changed LKAS FusionState to 1, that field was copied
into host LKAS, and FCA_Failinfo subsequently became 3. The fix enabled the
existing camera-side MDPS12 rewrite (ToiActive=0, ToiUnavail=1) for KIA_K7_PE;
it did not change MDPS feedback on the vehicle bus. HYUNDAI_ELANTRA is not in
the current send_mdps12 selection.

This Elantra segment does have the same unmodified feedback path: all 5,997
bus2 MDPS12 TX echoes match contemporaneous original bus0 payloads, with no
host-generated MDPS12. However, the distinguishing K7 fault sequence is absent:

- Stock camera LKAS FusionState stays 0 in all 5,997 frames; host LKAS and all
  5,994 bus0 TX echoes likewise have FusionState=0.
- Camera SysWarning and FcwSysState stay 0. Camera LdwsSysState=15 and
  ToiFlt=1 are already present at the segment boundary, with two brief clear
  windows later; they do not newly rise before the nine-second MDPS fault.
  Host LKAS replaces these with state 3/4 and its own fault/request fields.
- All 2,998 original bus2 FCA11 frames and corresponding bus0 TX echoes have
  FCA_Failinfo=0, FCA_Status=2 and VSM warning=0.
- All 2,998 original MDPS11 frames have warning lamp=0 and status=1.

Thus K7-style feedback compatibility is a relevant comparison, but the K7
FusionState-to-FCA propagation mechanism is not demonstrated here. The camera
is already reporting a non-normal LKAS state at the boundary, so earlier
segments remain useful to investigate its onset. Do not infer that adding
Elantra to the K7 rewrite list fixes the vehicle-side MDPS fault. The earlier
exclusion of stock LKAS insertion concerned whole-message forwarding, not
every possible camera feedback interaction.

## Follow-up: counters, checksums and delay

`counter_check.py` independently rereads the raw log. Every adjacent counter
step is +1 modulo its defined width for host LKAS11 (5,997), bus0 LKAS TX
echo (5,994), stock camera LKAS11 (5,997), original MDPS12 (5,997), SAS11
(5,997), MDPS11 (2,998), and their recorded camera-side forwarding echoes.
There are zero repeats, skips or reversals in these sequences. LKAS has 375
normal 15-to-0 wraps in each recorded stream; original MDPS12 has 23 normal
255-to-0 wraps. Additive checksum errors are zero for all checked LKAS11 and
MDPS12 streams; this statement does not extend checksum validation to SAS11
or MDPS11.

At first MDPS fault, bus0 LKAS echoes progress 9,10,11,12,13,14,15,0 and MDPS12
progresses 134,135,136,137,138,139,140,141. The fault appears at MDPS counter
137, not at its counter wrap. All LKAS echoes remain byte-identical to the
corresponding host packets in consecutive order.

| Observed timing metric | Entire segment | Last 0.5 s before first fault |
| --- | --- | --- |
| Mean host LKAS interval | 10.003 ms | 9.977 ms |
| Maximum host LKAS interval | 31.141 ms | 20.519 ms |
| Maximum LKAS TX-echo batch interval | 39.586 ms | 20.572 ms |
| Mean host-to-echo timestamp difference | 5.409 ms | 5.290 ms |
| Maximum host-to-echo timestamp difference | 16.081 ms | 11.215 ms |

The pre-fault 20.519 ms host gap occurs at 8.710468–8.730987 s, about 277 ms
before fault reporting. Near onset, host timestamps 8.985092, 8.993298 and
9.001369 s yield echoes 8.988221, 8.996672 and 9.008233 s. The next host gap,
9.001369–9.022300 s (20.932 ms), is still in progress when the first MDPS fault
is already received at 9.008233 s. Its full duration must not be treated as
a preceding 20.9 ms silence that triggered the initial fault.

The largest host gap (31.141 ms) and echo interval (39.586 ms) occur around
50.5 s without a new recorded temporary steering fault. Timing variation is
real, but no unique pre-onset delay explains this incident. Host publication,
hardware-submission echo and batched receipt timestamps cannot establish exact
wire delivery intervals or MDPS internal reception/timeout behavior. Thus the
log supplies strong negative evidence for malformed counters/checksums, but
does not fully exclude ECU-side timing sensitivity.

## Follow-up: CAN-FD pacing queue versus classic CAN

The user recalled a prior CAN-FD timing fix using a safety-layer queue and
counter handling. Current source confirms that CAMERA_SCC CAN-FD control
forwarding buffers accepted host packets, starts with two queued packets,
retains up to six, and dequeues on corresponding original RX. When empty it
can reuse the last body twice. Every substituted output takes the current
original RX counter and recalculates the checksum; body reuse does not mean
repeating the previous transmitted counter. This is distinct from the newer
cluster latest-value cache. Source includes prefill/headroom commits
`01a17f1482` and `036435548a` from 2026-08-14.

Current classic Hyundai LKAS uses immediate accepted-host transmission plus
a 200 ms stock-forwarding suppression timer. It has the normal CAN driver TX
queue, but no equivalent original-RX-paced LKAS buffering/reuse/counter rewrite.
The 200 ms timer is not an EPS acceptance deadline or a jitter correction.

The incident contains concrete host publication gap/catch-up pairs before
the first MDPS fault:

| Three successive host publication times | Two successive intervals |
| --- | --- |
| 8.502534 / 8.519029 / 8.523237 s | 16.495 / 4.208 ms |
| 8.610680 / 8.629373 / 8.634348 s | 18.694 / 4.975 ms |
| 8.710468 / 8.730987 / 8.736167 s | 20.519 / 5.180 ms |
| 8.901949 / 8.919897 / 8.924685 s | 17.948 / 4.788 ms |

In the last 0.5 s before the first fault, five recorded CAN batches contain
two LKAS TX echoes but only one stock LKAS and one original MDPS frame. All
echo counters remain consecutive. This supports investigating uneven host
command timing separately from duplicate/missing counters; it does not yield
the exact wire spacing between the two echoes or establish an EPS timeout.

The appropriate remaining hypothesis is delayed/clustered command delivery
despite valid counters, not an observed duplicate-counter defect. Similar
timing elsewhere without a fault does not prove timing harmless, because ECU
state/history may matter. Conversely, the structural similarity to the earlier
CAN-FD problem does not prove this warning's cause or authorize assuming a
queue fixes it. A pacing change would affect command age, stale-command reuse,
disengagement latency and the existing two-frame request interruption; these
semantics need explicit validation before any vehicle deployment. No code or
settings changes were made during this comparison.

## Reproduction evidence

Follow-up on periodic request interruption: the original initData Params
explicitly contains `MaxAngleFrames=89` (not 87). Current code counts frames
only while lateral actuation is requested and absolute steering angle is at
least 85 degrees. It sends two frames with ActToi=0/ToiFlt=1 after 89 qualifying
frames, retaining the torque field. The recorded revision is still unavailable
locally, but actual host CAN confirms exactly two such pulse runs: 41.995003 /
42.006852 s (torque -82/-84) and 43.262133 /43.274206 s (-65/-62).
There is no such pulse before the nine-second warning. The nine-second
fault-response interruption instead has ActToi=0/ToiFlt=0 and zero torque.
Thus that warning was not preceded by the periodic fault-avoidance pulse.

Private scripts, metadata and reduced text outputs are retained under
`.analysis/archive/2026-10-03/29b-steering/`. Scripts use the full cereal schema
and decode relevant little-endian CAN signals from the repository DBC.
Generated full-stream JSON is reproducible from the supplied raw file and was
removed after analysis. Do not commit vehicle captures or settings snapshots.
