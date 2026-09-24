# EV5 cluster corner-object visibility (2026-09-24)

## Findings

Full-cereal decoding of EV5 segments `000000f3--f2596b8cdb--69` and
`000000f3--f2596b8cdb--97` separates two cases: missing display geometry and
present geometry carrying a hidden ccNC object type.

Both logs record `dc2d92cab69f03c00d06c67074b2cbb3578523f7` in
`initData.gitCommit`. The upload metadata supplied for segment 97 instead names
`2bc1f8c1`. They belong to the same running route; the upload's current checkout
commit is not evidence that the driving processes restarted into that version.
An update followed by no process restart explains this difference. Segment 97
must not be presented as vehicle validation of `2bc1f8c1`.

| Evidence | Segment 69 | Segment 97 |
| --- | --- | --- |
| Camera-bus `0x162` packets | 1,199 | 1,202 |
| LF/RF/LR/RR `0x162` DETECT | All zero | All zero |
| Nonzero corner distance | None | RF: 130 packets; RR: 156 packets |
| `0x1EA` corner information | All zero | Matching RF/RR geometry and DETECT=1 |
| Transmitted `0x162` packets | 1,200 | 1,201 |
| Transmitted front FF type | 4 in all 1,200 | 4 in 252; 0 in 949 |
| Transmitted corner types | All zero | All zero, despite RF/RR distances |
| `liveTracks` frames with `corner235` points | 1,200 / 1,200 | 1,202 / 1,202 |

Segment 97's transmitted RF distances are 36.4–38.8 m at approximately
11.44–12.19 seconds and 5.7–15.5 m at 20.40–26.05 seconds. RR distances are
0.2–18.5 m at 0.54–8.29 seconds. Times are relative to the first `can` event,
not the route's boot-time `initData` timestamp. The ccNC DETECT value remains
zero throughout those intervals, while `0x1EA` marks the objects visible.

These are not merely a loss of radar points or blocked CAN transmissions:
corner points remain in `liveTracks`, the relevant received/transmitted display
messages have valid checksums, and bus-128 transmit receipts contain the same
hidden corner types. Receipts establish bus transmission, not the dashboard's
actual rendering. No dashboard video is available in these uploads.

The boot Params in both logs are `EnableCornerRadar=2`, `HyundaiCameraSCC=2`,
`EnableRadarTracks=3`, `CanfdDebug=0`, and `PaddleMode=0`. The nearest preceding
September 23 settings snapshot and newest September 24 snapshot agree on these
values. The code also sends the existing `0x4B9` activation sequence; that does
not prove OEM object generation is active in every driving condition.

An earlier same-vehicle segment, `000000ed--42bf5941d2--22` on `c1849b50`, also
has nonzero geometry with zero ccNC corner DETECT. This condition predates the
September 23 restoration; that restoration is not established as its cause.

## Correction and scope

`1fbfe33156` incorrectly restored old box semantics (`1`) rather than the
requested car presentation. `2bc1f8c1` corrected the car type to `3` but still
required incoming DETECT > 0. That condition leaves the EV5's distance-bearing,
hidden objects invisible, even after restarting into that version.

For ccNC `0x162`, nonzero LF/RF/LR/RR distance now sets the corresponding DETECT
to `3` (gray car), including incoming DETECT=0. Zero-distance values retain
their incoming type. `0x1EA` keeps its separate existing normalization because
its enumerations differ. Front FF lead selection/presentation, received
geometry, scheduling, activation messages, and radar processing are unchanged.
Blinking and the old rear distance clamp remain removed.

This repairs the missing display request in segment 97. It cannot synthesize
side/rear display objects in segment 69, where both messages' geometry is zero.
Determining why the OEM display-object output is empty there requires separate
evidence; do not equate it with corner radar hardware being inactive.

## Validation

- 190 focused CCNC cluster, lead, and fault-filter tests pass with a Windows
  Params bootstrap. A distance-bearing DETECT=0 regression fails before the fix.
  Tests decode real CAN packets and check the DBC `GRAY_CAR` meaning.
- Repacking 7,202 recorded `0x162`/`0x1EA` transmit packets across the three
  segments changes only the intended ccNC corner types and checksum fields.
  Geometry, front-lead fields, and other signal values are preserved.
- Segment 69 changes no object types. Segment 97 changes 130 RF and 156 RR
  slots across 286 packets. The earlier segment changes 845 corner slots across
  320 packets. All repacked checksums validate; `0x1EA` object types stay intact.

These are offline CAN-output checks, not confirmation of physical dashboard
visibility. Local scripts, summaries, and captures are retained in the ignored
analysis archive, not in Git. This display-only correction does not change
radar detection, lead selection, or NAS replay dependencies.

## Follow-up: f4 segment 1 and corner activation

`000000f4--ef10fb59ef--1` covers September 24, 14:05:49.2 through
14:06:49.1 KST by GPS. Upload metadata names `a92e6ffc`, but initData and
pandad log context name `2bc1f8c1`. Unlike the two f3 segments, this capture
does show a fresh startup: initData monotonic time is about 108 seconds,
and Panda uptime is 62 through 123 seconds. Do not tell the user that no
restart occurred. The recorded startup version still does not validate the
latest fix; initData reads the manager's stored GitCommit parameter rather
than independently identifying every running module.

Both incoming `0x162` and `0x1EA` have 1,200 packets with all four corner
types, distances, and lateral positions zero. The 1,198 outgoing packets
of each message retain that empty geometry, have valid checksums, and
appear in bus-128 transmit receipts. Applying the `a92e6ffc` distance-only
normalization cannot change these zero-distance corner fields either.
leadOne and leadTwo are absent in all 1,200 radarState messages; outgoing
front FF type and 0x161 TARGET are also zero throughout. Corner radar itself
is not silent: 1,086 corner235 points occur in 1,027 liveTracks frames, and
leadRight is present in 160 radarState messages. Those tracks are not used
to synthesize the OEM corner display slots.

`EnableCornerRadar=2` remains enabled. The controller sends the existing
0x4B9 activation sequence 72 times on camera bus 2, but incoming
HDA_CntrlModSta and HDA_MODE2 remain zero throughout. The setting gates this
request with `> 0`; changing 2 to 1 would not change that activation path.
HDA_OptUsmSta is 2 (function on), so the option being enabled and the
runtime HDA state being active must be distinguished.

Comparison of incoming 0x1EA packets makes the association concrete:

| Segment | HDA_MODE2 and corner geometry |
| --- | --- |
| f3--69 | 1,200 packets: mode 0, no geometry |
| f3--97 | 284: mode 2 with geometry; 62: mode 2 without; 854: mode 0 without; 2: mode 0 with |
| ed--22 | 273: mode 2 with geometry; 45: mode 1 with; 879: mode 0 without; 1: mode 0 with |
| f4--1 | 1,200 packets: mode 0, no geometry |

This implicates OEM display-object availability/activation as a separate
condition from DETECT normalization. It does not establish why the OEM
state did not activate, or prove a mode bit is sufficient to enable it.
Do not force that bit or invent distances on this evidence. The captured
Panda registerDivergent fault also occurs in the comparison segments with
geometry; safetyTxBlocked stays zero in f4--1. It is not established as the
cause of the missing corner display. No runtime behavior was changed for
this follow-up; raw logs, comparison scripts and summaries remain private
in the local analysis archive.

### Stock LFA/SCC overlap, from camera-bus input

The following uses only `can` packets with `src=2`: SCC_CONTROL `0x1A0`
(MainMode_ACC/ACCMode) and LFAHDA_CLUSTER `0x1E0`
(HDA_LFA_SymSta/HDA_CntrlModSta). It does not infer stock state from
`sendcan`, bus-0 display overrides, or carState.cruiseState. With openpilot
longitudinal control, carState.cruiseState.enabled comes from TCS ACC_REQ,
not the camera's SCC_CONTROL state.

For f4--1, relative to the first CAN event:

| Time (s) | Received stock LFA | Received stock SCC |
| --- | --- | --- |
| 0 to 23.38 | Off (0) | Main=0, ACCMode=0 |
| 23.38 to 27.68 | Green (2) | Main=0, ACCMode=0 |
| 27.68 to 28.68 | Green blink (3) | Main=0, ACCMode=0 |
| 28.68 to 30.59 | Off (0) | Main=0, ACCMode=0 |
| 30.59 to 36.28 | Off (0) | Main=1, ACCMode=1 (enabled) |
| 36.28 to 39.68 | Off (0) | Main=0, ACCMode=0 |
| 39.68 to 41.04 | Off (0) | Main=1, ACCMode=1 (enabled) |
| 41.04 to 57.55 | Off (0) | Main=1, ACCMode=2 (driver override) |
| 57.55 to segment end | Off (0) | Main=0, ACCMode=0 |

There is no overlap between green/blinking stock LFA and enabled/override
stock SCC in this segment. HDA stays zero throughout. By comparison, f3--97
has stock SCC enabled from 0.44 to 26.00 seconds; stock LFA is green until
12.18 seconds and again from 20.33 seconds. HDA becomes 2 at 0.53 seconds,
drops at 12.23 seconds as LFA blinks, returns at 20.37 seconds, and drops at
26.08 seconds after SCC switches off. That sequence supports checking the
stock LFA/SCC prerequisites before changing the 0x4B9 request cadence.

The f4--1 log also contains 135 outgoing camera-bus 0x1AA packets with
LFA_BTN=1 and six with ADAPTIVE_CRUISE_MAIN_BTN=1. These counts are packets,
not distinct button presses, and do not establish successful activation.
The stock LFA response remains off during every stock SCC-active interval.
No activation or vehicle-control behavior was changed for this analysis.

### Carrot control state against stock LFA/SCC

Carrot's own state comes from carControl.enabled, latActive and longActive.
Join each sample to the latest past bus-2 SCC/LFA sample within 150 ms;
f4--1 has no missing/stale SCC or LFA samples in this comparison. The
times below share the first-CAN origin used above.

| f4--1 time (s) | Carrot enabled | latActive | longActive |
| --- | --- | --- | --- |
| 0.01 to 23.23 | false | true | false |
| 23.23 to 27.51 | false | false | false |
| 27.51 to 40.70 | false | true | false |
| 40.70 to 57.59 | true | true | false |
| 57.59 to 59.90 | false | true | false |

During all 1,689 samples with Carrot enabled, stock LFA is off. Stock SCC
is enabled in 35 of those samples, in driver override in 1,651, and off in
the last three samples just before Carrot disables. Thus Carrot cruise and
steering being enabled does not imply that stock LFA is enabled alongside
stock SCC. In contrast, stock LFA is green for most of the interval where
Carrot latActive is false (23.39 to 27.51 seconds on the joined timeline).
Short state-transition delays must not be interpreted as continuous overlap.

For those same 1,689 enabled samples, carState.gasPressed is true,
selfdriveState.state is overriding, and onroadEvents contains
gasPressedOverride with overrideLongitudinal. This explains why longActive
remains false despite enabled=true; the entire f4--1 segment has no active
Carrot longitudinal control. It does not mean Carrot cruise was never enabled.

The prior f3--97 comparison also has a mismatch, on the other axis: stock
LFA stays green while Carrot cruise is enabled from 29.43 seconds onward,
but stock SCC remains MainMode_ACC=0/ACCMode=0. Earlier HDA-active intervals
(about 0.53 to 12.23 and 20.37 to 26.08 seconds) instead occur while Carrot
enabled is false and stock LFA/SCC overlap. These observations support
investigating stock activation/synchronization, rather than treating Carrot
engagement flags or outgoing cluster icons as stock-state confirmation.
They do not yet identify why an individual button request fails or a stock
function switches off. No runtime code changed in this comparison.

### Automatic LFA button requests versus received button input

The f4--1 retry loop is running: outgoing `sendcan` bus-2 `0x1AA` carries
27 distinct LFA_BTN pulses, each five packets (135 asserted packets total),
at roughly two-second intervals while received stock LFA is off. All 27
pulses also occur in `can` bus 130 (Panda's bus-2 transmit receipts), with
valid 0x1AA checksums. No stock LFA activation follows those automatic
pulses. This is not merely a failure to enter the retry branch or enqueue
the message.

Received vehicle button input instead changes on `can` bus-0 `0x10B`
(CRUISE_BUTTONS_ALT2); incoming bus-0 `0x1AA` LFA_BTN stays zero. The
0x10B presses are also present in bus-130 forwarding receipts:

| Segment | Received 0x10B LFA press (s) | Subsequent stock bus-2 LFA state |
| --- | --- | --- |
| f4--1 | 23.057 to 23.179 | Green at 23.380 |
| f4--1 | 27.381 to 27.458 | Blink at 27.680, off at 28.681 |
| f3--97 | 11.856 to 11.977 | Blink at 12.177, off at 13.225 |
| f3--97 | 20.102 to 20.144 | Green at 20.331 |

In f3--97 four automatic 0x1AA pulses at about 13.51, 15.51, 17.51 and
19.51 seconds fail to restore LFA; the received 0x10B press at 20.10 seconds
precedes its recovery. This repeats the distinction in a second segment.

CarState already prefers the available CRUISE_BUTTONS_ALT2 for interpreting
driver buttons. However, the automatic LFA request in create_ccnc_messages
still packs CS.cruise_btns_msg_canfd, which selects CRUISE_BUTTONS_ALT
(0x1AA) for this vehicle. The protocol mismatch is therefore a concrete
candidate for ineffective automatic activation. The evidence establishes
that the current retry does not achieve activation in these captures; it
does not yet validate a replacement 0x10B transmitter or all of that
message's integrity/forwarding requirements. No transmitter or Panda
safety behavior was changed in this investigation.

## Correction: native ALT2 automatic button requests

For camera-SCC longitudinal control with an observed CRUISE_BUTTONS_ALT2,
create_ccnc_messages now sends automatic LFA/SCC button intent using 0x10B
instead of the inactive 0x1AA button layout. Vehicles without ALT2 retain
their existing path. The sparse 0x10B DBC is unchanged: the host request
contains only button intent and a verified Hyundai CAN-FD checksum. It is
not suitable for direct vehicle transmission.

Panda admits this request only on bus 2 with length 16, camera-SCC and
longitudinal safety enabled, valid checksum, and one of neutral, LFA,
SET, or MAIN. The TX hook consumes it using the existing buffered-for-forward
mechanism; can_send does not put the sparse host packet onto CAN. The
forwarding hook applies fresh intent to the original bus-0 0x10B, changing
only byte 10's button bits and the checksum. Its original counter (which
advances by two per received frame in these captures), reserved/unknown
fields, input cadence and destination remain intact. It emits no frames
without original vehicle input.

Physical buttons take precedence in both host and Panda, including CANCEL
and unrecognized nonzero cruise codes. Invalid original checksums are not
repaired. A host neutral request releases immediately; a missing refresh
expires after 120 ms, and even continuous refresh cannot hold a press for
200 ms. A neutral request is required to rearm after timeout, overlong
press, conflicting request or physical input. Safety initialization clears
all pending intent. The existing camera-SCC steering, longitudinal limits,
generic buffered forwarding and radar processing are unchanged.

LFA retry remains approximately every two seconds while the received stock
LFA state is off, and stops for any nonzero state. Missing LFA state is not
treated as off. SCC MAIN/SET activation retains the Carrot-enabled,
speed and brake-hold/parking-brake guards; its pulse follows the LFA pulse
with a release interval. Stock SCC that is already active is no longer
toggled with MAIN just because HDA is off on this ALT2 path. Existing
standstill-resume/stopping behavior is retained.

Validation:

- 257 focused tests pass: existing CCNC display/lead/fault tests, new host
  request tests, and a native C harness that compiles the production Panda
  safety TX/forward hooks. Tests cover allowed/rejected modes and buses,
  driver priority, release, expiry, bounded press, reset, checksum failures,
  counter/unknown-bit preservation and timer wraparound.
- Compiling the previous and modified Panda code with safetyParam 189 shows
  the same valid host 0x10B request rejected before and accepted/consumed
  after. Merely changing the Python address would indeed have been blocked.
- Offline replay invokes the new Python request helper and compiled C hooks
  against both full logs. All 3,003 original 0x10B frames are forwarded;
  all 63 physical-button frames remain byte-identical. f4--1 modifies 66
  idle frames to LFA requests; f3--97 modifies 11 to LFA and 16 to MAIN.
  Every output checksum validates; only button bits and checksum change.
  Recorded stock responses are held fixed: this is not a simulation of
  successful activation or proof that the dashboard now renders cars.

The update includes Panda firmware source, so the device must complete its
normal build/Panda firmware update and restart. Old Panda firmware rejects
the new host request. Native desktop compilation/replay does not establish
an on-device firmware build or successful EV5 activation. A follow-up route
must verify bus-130 0x10B button pulses, bus-2 LFA/SCC overlap, and the return
of OEM corner display geometry. No vehicle operation was performed here.

## Follow-up: reminder when downloaded code is not running

The manager now snapshots the checkout commit during initialization, while the
launcher's boot checkout lock is still held. It keeps that identity across
ignition cycles and checks the local checkout every five seconds in the
normal-priority manager. Two matching changed-commit reads set
`managerState.rebootRequired`. Missing/failed reads do not create a warning.
Packaged installations without `.git` use the commit in `build.json` instead.
Remote-only fetches, branch renames at the same commit, and uncommitted edits
do not constitute a downloaded-commit mismatch.

After 15 seconds of an initialized onroad session, selfdrived shows one
eight-second NNFF-style prompt: "Reboot to Apply Update" / "Park safely before
rebooting your device". Korean text is "업데이트 적용을 위해 재부팅하세요" /
"안전한 곳에 주차한 뒤 재부팅하세요". It is informational: it neither blocks
engagement nor disengages, and higher-priority alerts retain precedence.
It can also report an update detected later in the drive, if not already shown
in that session. A new onroad session repeats the reminder until the manager
restarts into the installed checkout. Replay/simulation do not generate it.

The feature itself must first be installed and restarted once; code already
running before its installation cannot retroactively produce this new alert.
Focused desktop tests cover real Git changes/worktrees, reverting/restarting,
read failures, alert timing, translations, and unchanged engagement state.
Device display/timing validation remains separate from these tests.
