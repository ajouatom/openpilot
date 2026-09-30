# Ioniq 5 PE intermittent driver-assistance cluster warning

## Scope

The user supplied a photo of the OEM cluster displaying "Check Driver
Assistance system", reporting one brief occurrence around ten seconds into
route `000005ca--e39250b8f6--3`. Full rlogs for segments 3 and 4 were decoded
with the complete cereal schema. Both identify HYUNDAI_IONIQ_5_PE, mici/C4,
clean revision `a315e806b03b86c2f97f720c137f6595fa046244` on carrot-wip.
The incident revision DBC and controller source were used.

Times below are relative to the first carState in segment 3
(monotonic 239.716695839 s). Segment 4 starts at 299.675608874 s.
Only these two segments are present for this route in the uploaded vehicle
directory; the preceding onset/history cannot be reconstructed from them.

## Findings

The photographed warning is real user evidence, but its initiating CAN
request or ECU reason is **not identified** in these captures. In particular,
this is not a demonstrated recurrence of the GV70 warning signature.

| Evidence | Segment 3 | Segment 4 |
| --- | --- | --- |
| Original camera CCNC_0x162 frames | 1,199 | 1,201 |
| Outgoing CCNC_0x162 frames | 1,199 | 1,200 |
| FAULT_DAS and all other decoded 0x162 fault fields | Always 0, RX and TX | Always 0, RX and TX |
| Original MDPS LKA_FAULT / LFA2_FAULT | Always 0 | Always 0 |
| Original SCC SysFailState | Always 0 | Always 0 |
| Camera FCA_SYSWARN / cluster HDA_InfoPUDis | Always 0 | Always 0 |
| carState CAN validity | 5,996 / 5,996 valid | 6,000 / 6,000 valid |
| carControl enabled / latActive | Always true | Always true |
| Panda safetyTxBlocked | Always 0 | Always 0 |

The DBC associates `CCNC_0x162.FAULT_DAS=1` with the photographed wording.
All original RX, sendcan and bus-128 transmit-echo payloads have zero in the
entire 39-bit fault field region, with valid independently calculated CRCs.
Transmit echoes match sendcan byte-for-byte in both segments. This rules out
a nonzero request in this decoded message during the capture; it does not
rule out cluster-local fault handling, an earlier latched indication, or
another unidentified request path. A transmit echo does not prove cluster
application-level acceptance or recover its internal diagnostic state.

At +8..12 s, all inspected fault/popup fields remain zero. Longitudinal
control is overridden by the accelerator, while lateral control stays active.
There is no accompanying CAN timeout, EPS fault, safety block, modelV2 or
cameraOdometry invalidity. Both segments have 1,200 livePose samples, all with
inputsOK, sensorsOK and posenetOK true. No new CAN hardware error count appears;
the pre-existing bus-1 error/reset counts and SPI checksum count remain constant.

A separate Openpilot `steerSaturated/warning` occurs at +4.191..6.361 s,
displaying "take control / turn exceeds limit" with promptRepeat. This occurs
during a low-speed turn and is not the photographed service-check wording.
Camera ADRV_0x161 has ALERTS_5=2 (watch surrounding vehicles) at
+2.423..6.425 s and ALERTS_2=1 (hands on wheel) at +23.366..27.365 s;
both fields remain zero in the outgoing copies. These do not establish the
cause of the service-check popup.

## Separate counter finding and timing limitation

Outgoing 0x162 reuses the latest received COUNTER rather than assigning an
independent transmit sequence. Segment 3 has 65 repeated counter steps and
66 steps of +2, versus consecutive original RX counters. These occur at
+23.766..41.862 s; all CRCs remain correct. Segment 4 TX counters are
consecutive. The source retains COUNTER in the copied dictionary at the
CCNC_0x162 packing call, explaining the transmission observation.

This is a concrete protocol investigation candidate, **not a proved cause**
of the reported ten-second event: its first occurrence is substantially later.
The photo shows 45 mph with a 44 mph set speed, whereas logged +10 s is about
26.7 mph with a 42.9 mph set speed. Values close to the photo occur around
+18..26 s. Without a synchronized photo timestamp, these cannot be aligned to
a single CAN frame or used to infer causality from overlap.

No runtime or warning-suppression changes were made. The next discriminating
evidence is an accurately timed recurrence with preceding logs and, if
available, the relevant ECU/cluster diagnostic reason. Do not expand the
GV70 suppression to this platform based on this photo alone.

Private reproduction scripts, compact results and source locations are under
`.analysis/archive/2026-09-30/ioniq5-cluster/`. Raw logs and decoded private
payloads are not committed. Offline decoding does not establish vehicle fault
absence or a verified fix.

## Follow-up: transmission interruption near ten seconds

The user reaffirmed the occurrence near +10 s and specifically asked whether
CAN transmission stopped. Analysis therefore uses +8..12 s irrespective of
the photo-speed alignment limitation above.

| Stream | Frames in four seconds | Maximum host-log interval |
| --- | --- | --- |
| sendcan envelopes / angle steering command | 400 | 18.005 ms |
| CAN receive envelopes | 400 | 17.376 ms |
| Each cluster TX: 0x161, 0x162, 0x1e0, 0x1ea, 0x200 | 80 | 54.815 ms |
| Each corresponding cluster bus-128 echo | 80 | 61.811 ms |
| Angle command 0xcb bus-128 echo | 400 | 23.022 ms |
| SCC 0x1a0 bus-128 echo | 200 | 33.777 ms |

Every cluster sendcan payload in this window has a byte-identical Panda
returned echo, with the longest host-published sendcan-to-echo interval
15.364 ms. No rejected frames appear. All five buffered steering/SCC/feedback
streams (0xcb, 0x12a, 0x1a0, 0xea, 0x175) have consecutive original RX and
returned-echo counters, with valid independent CRC checks. Actual returned
angle-command LKAS_ANGLE_ACTIVE stays 2 in all 400 frames, whereas original
camera commands stay 1; no switch to the inactive stock command is observed.
The 35 pandaStates samples in this window show no safety block, invalid safety
RX, buffer overflow, bus-off, or increase in CAN error/loss/reset counters.

There is minor host-command buffering activity, not an observed transmission
gap. Panda serial diagnostics are published at +8.126 s for MDPS (0xea) and
angle command (0xcb), and at +9.611 s for LFA (0x12a). Each reports one reuse of
the previous command, `exhausted:0, q:1`. The 0x12a report records maximum
host-push age 8,582 us and inter-push interval 18,305 us. The buffered path
waits for two queued commands before restarting consumption and permits up to
two reuses. Reused payloads take the original incoming counter and a new CRC,
so counter continuity alone would not detect reuse. Serial publication time
is not a precise wire timestamp; the reports are also rate-limited aggregates.
Their cumulative software queue-overflow counters (6 or 7) are unchanged in
the next reports and must not be described as new hardware TX losses at +10 s.
These isolated reuses do not establish the cause of the cluster warning.

Interpretation limits are material: in this H7 firmware, a returned echo is
created immediately after writing TXBAR, when the packet is submitted to the
hardware transmit FIFO. It is **not** per-frame physical ACK/completion proof,
nor proof that the cluster accepted the payload. The intervals above are
host publication intervals, including receive batching, not oscilloscope bus
timing. Buffered control frames also have their counters/CRCs rewritten in
Panda, so byte-for-byte host/echo matching is only used for the direct cluster
streams. Sparse navigation/event messages and the deliberately burst-scheduled
outgoing 0x4b9 are not treated as periodic transmission failures.

Conclusion: no host/Panda transmission-stream interruption is observed around
the reported event. A cluster-internal receive/interpretation fault is not
excluded by this log. No runtime change was made. Reproduction and compact
results: `.analysis/archive/2026-09-30/ioniq5-can-gap/`.

## Comparison with the previous reported warning

The previous same-vehicle investigation was recovered from
`.analysis/archive/2026-09-29/cluster-error-7f419ed56030e135/README.md`.
Its full logs are `000005c5--55be38ea85--0` and `--1`, clean commit
`26706b95698c28fce9ff95efb1ae5e507680d5fb`. Both were reread alongside the
current segments 3/4 with the full cereal schema. The buffered-forwarding
safety header and H7 FDCAN driver have no source differences between these
two incident revisions. This is a source comparison, not an independent
firmware-binary attestation.

All times in this comparison start at each segment's first carState. The
previous segment 0's first CAN is 1.323491 s earlier, so times from the older
CAN-anchored report differ by that amount.

| Capture | Reuse diagnostics published near the early part of the segment | Later observations |
| --- | --- | --- |
| Previous c5/0 | MDPS +7.602 s; angle command +8.062 s; LFA +11.466 s, each count=1/exhausted=0 | At +40.915..40.916 s MDPS/angle/LFA each report count=2/exhausted=1 |
| Previous c5/1 | No reuse diagnostic reported | Serial logging exists; absence of a report is not per-frame proof that no reuse occurred |
| Current ca/3 | MDPS and angle +8.126 s; LFA +9.611 s, each count=1/exhausted=0 | Further MDPS/angle/LFA reuses at +27.415 s |
| Current ca/4 | MDPS/angle +4.044 s; LFA +11.296 s, each count=1/exhausted=0 | Further reuses near +56.910 s |

Thus reuse is present in both reported-warning captures, including an early
LFA reuse near the current user-reported warning time. It is not unique to
that occurrence. The user reported only one current popup; ca/4 is a useful
unreported comparison interval, not independently verified video evidence
of no warning. The previous popup's exact timestamp is still unspecified.
No causal timing claim can be made for its +11.466 or +40.916 s activity.

The previous segment 0 also contains a distinct SPI disturbance:
spiChecksumErrorCount is 1 initially, 7 by +40.672 s and 21 by +40.998 s.
The +40.915 s reuse reports consume the second allowed reuse, but this is not
itself evidence of a subsequent fallback or dropped CAN frame. In +/-250 ms
windows around the inspected early and later reuse reports, returned angle
commands stay active (LKAS_ANGLE_ACTIVE=2), original camera commands remain 1,
and inspected returned-stream counters remain consecutive. The largest
host-published returned gap around +40.916 s is 42.666 ms. Current ca/3 and
ca/4 retain spiChecksumErrorCount=1 throughout, so that SPI disturbance is not
a shared observed condition of the two incidents.

### Diagnostic-code comparison

The previous c5/0 response at +50.294669 s from 0x738 on bus 2 is
`07 59 02 89 68 c5 86 08`; c5/1 response at +27.996818 s from 0x7cc on bus 0
is `07 59 02 0d 68 c5 86 08`. Both contain DTC C28C5:86, status 0x08
(confirmedDTC set, testFailed clear). Its manufacturer-specific definition,
creation time and relationship to the photographed warning remain unknown.

Current ca/3 at +28.086707 s returns `03 59 02 89 aa aa aa aa` from 0x738;
ca/4 at +6.046391 s returns `03 59 02 0d aa aa aa aa` from 0x7cc. All these
reads follow the same `19 02 0d` request. The current positive responses
contain no DTC entries matching that requested status mask. This is stronger
than merely failing to log a diagnostic service, but does not mean every
possible ECU fault is absent, or prove that any earlier DTC was cleared.

### Further discriminating evidence

1. Correlate a cluster recording or passenger-created incident mark with the
   host monotonic timeline. Retain the preceding and following segments, and
   record non-warning intervals containing reuse as comparisons.
2. If adding diagnostics, use bounded event storage in Panda for the original
   MCU timestamp, address/destination, host-command sequence, queue depth,
   last fresh command age, reuse index, and whether original-frame fallback
   actually occurred. Drain at a bounded rate; do not print synchronously on
   every reuse or change buffer/control behavior. Existing serial summaries
   have neither per-frame identity nor exact host/MCU clock alignment.
3. Compare pre/post-incident read-only ECU DTC status and supported freeze-frame
   or extended records, including the cluster, while parked. Preserve records
   before any clearing. Manufacturer diagnostic documentation is needed to
   identify C28C5:86's specific monitored signal. No active diagnostic command
   was sent to a vehicle during this task.

No warning suppression, buffer policy, control change, or new firmware logging
was deployed. Compact results and reproducible scripts are archived at
`.analysis/archive/2026-09-30/ioniq5-reuse-compare/`.

### Checksum and unchanged-payload follow-up

The user specifically questioned whether reuse produces checksum failures.
`canfd_apply_counter_and_update_checksum` replaces the counter with the current
original-RX counter and then recalculates the Hyundai message checksum. The
earlier SPI checksum diagnostic is a separate host/Panda transport error,
not the checksum embedded in vehicle CAN payloads.

Independent CRC16 calculation over the returned payloads for eight addresses
(0xcb, 0x12a, 0xea, 0x175, 0x1a0, 0x161, 0x162, 0x1e0) found:

| Segment | Returned frames checked | Bad Hyundai payload CRC | Frames within +/-250 ms of reuse reports / bad CRC |
| --- | --- | --- | --- |
| c5/0 | 24,797 | 0 | 1,358 / 0 |
| c5/1 | 27,597 | 0 | No reuse report window |
| ca/3 | 27,591 | 0 | 1,350 / 0 |
| ca/4 | 27,610 | 0 | 918 / 0 |

Total: 107,595 returned frames, zero invalid payload checksums. This checks
the Hyundai application checksum in the submitted CAN data, not the CAN
controller's physical-link CRC or receiving ECU's acceptance.

More specifically, in +/-500 ms windows around c5/0 +11.466436 s, ca/3
+9.611457 s and ca/4 +11.296011 s, every LFA (0x12a) host command and returned
frame has the same bytes after checksum/counter:
`18 80 00 00 00 00 0a 00 04 00 00 00 00` (100 host and 100 returned frames
per window). Thus the reported LFA reuse has no distinguishable old command
content in these windows: a fresh host LFA body would be identical. Together
with counter continuity and valid CRC, this weighs against that isolated LFA
reuse producing a corrupt or stale-value message. It does not exclude a timing
or cross-message interaction, a separate earlier angle/feedback event, or an
unobserved ECU/cluster diagnostic condition. Reuse must not be equated with
either a checksum error or a proven warning cause.

## Longer recurrence: same route, segment 32

The user supplied `000005ca--e39250b8f6--32` and reported a longer occurrence.
It is the same clean a315e806 revision and Hyundai CAN-FD safety parameter 157.
Full carState coverage spans 59.999 s from monotonic 1979.689016651 s.
The exact onset/end of the visible warning has not been synchronized to this
segment. Only segments 3, 4 and 32 of this route are present on the NAS;
preceding segment 31 is unavailable in the uploaded set.

- There are **no reuse diagnostics** among 500 parsed log/errorLog messages.
  There are also no Panda serial messages at all. This is absence of a report,
  not independent per-frame proof of zero reuse: diagnostic aggregation and
  serial delivery limits still apply, as does the unknown pre-segment history.
- All 13 original and transmitted CCNC_0x162 fault fields remain zero.
  Original MDPS LKA_FAULT/LFA2_FAULT, camera FCA_SYSWARN, SCC SysFailState and
  HDA_InfoPUDis remain zero. Each original cluster stream has 1,200 frames.
- All 1,200 host sends for each direct cluster stream (0x161, 0x162, 0x1e0,
  0x1ea, 0x200) have byte-identical returned echoes within 16.067 ms of host
  publication. The 1,201 returned frames per stream include one initial echo
  before the first logged host send. Maximum cluster host-send gap is
  57.983 ms and returned-echo gap 64.953 ms; these are host-log intervals.
- Independent checksum verification covers 27,611 returned frames across the
  same eight addresses as above: zero invalid Hyundai payload CRCs, and every
  adjacent returned counter step is +1. No rejected frames are recorded.
- All 6,001 carState samples have valid CAN and no steering/ACC fault. All
  6,004 carControl samples remain enabled, lateral active and longitudinal
  active. Model/odometry messages remain valid; all 1,200 livePose samples have
  inputsOK/sensorsOK/posenetOK true. Two brief radarCutin sounds at +33.299
  and +45.783 s are separate Openpilot events, not decoded OEM service popups.
- SPI checksum count is already **67 at segment start and stays 67** in all
  530 Panda state samples. It was 1 throughout segments 3/4. Therefore the
  cumulative count increased before segment 32, in the unprovided interval;
  this is not evidence of an SPI error during the long visible warning.
  CAN error/loss/reset counters do not increase and safety blocks, bus-off
  and hardware RX/TX buffer overflows remain zero in this segment.
- At +56.345788 s, 0x738 on bus 2 replies `03 59 02 89 aa aa aa aa` to
  `19 02 0d`: no DTC entries matching that mask. There is no 0x7c4/0x7cc
  camera-DTC query/response pair in this segment; do not infer its result.

The long recurrence has no accompanying reported reuse, message checksum
failure, counter discontinuity or host/Panda transmission interruption. This
weakens a simple immediate-reuse explanation, without excluding an earlier
trigger or an ECU/cluster-local condition. If the popup was already visible
at segment start, the preceding log is needed to locate its initiation and
to place the earlier SPI errors relative to it. No runtime change was made.

### Audio availability

FFmpeg inspection of the actual uploaded `qcamera.ts` reports a 60.00-second
MPEG-TS program containing one H.264 video stream (526x330, 20 fps), with no
audio stream. The full rlog also has no rawAudioData messages. This file
cannot supply recorded warning audio. Source `loggerd.h` includes audio in
qcamera.ts only when RecordAudio is enabled; the loggerd audio test checks
both the TS audio stream and rawAudioData recording for that parameter.
No recording setting was changed. Local audio/video metadata is archived
with the analysis, not the source media.

Private reproduction: `.analysis/archive/2026-09-30/ioniq5-cluster32/`.

## Photo alignment and repeated-occurrence clarification

The user subsequently supplied another photo, suggested approximately forty
seconds based on road appearance, and clarified that the warning appeared
several times. Treat occurrence counts and exact per-segment warning times as
unresolved. In particular, the earlier unreported ca/4 interval must not be
used as a confirmed warning-free negative control.

The new photo again shows "Check Driver Assistance system", with indicated
speed 30 mph, set speed 75 mph, a right-side concrete barrier, an overhead
sign and a dark SUV ahead. Sequential decoding of all 1,200 qcamera frames
places a visually similar sign/barrier/lead-vehicle arrangement around video
+50..53 s, rather than the van and roadside guardrail seen near +40 s. This
is visual scene matching, not a synchronized exact photograph timestamp or
the time at which the warning first appeared.

Video/host alignment was checked independently with qRoadEncodeIdx and decoded
PTS: frame 800 has PTS +40.001011 s, frame 1040 +52.002578 s and frame 1060
+53.002467 s. The first video PTS is monotonic 1979.608389 s, approximately
80.6 ms before the first carState. qRoadEncodeIdx timestampEof agrees with
these PTS to microsecond-scale container rounding; no seconds-long video/log
offset was found in this check.

Logged indicated speed near +40 s is about 34.2 mph, with set speed 74.6 mph.
Around +50..53 s, indicated speed is about 26.7..28.6 mph and the set speed
remains 74.6 mph; around +54 s it reaches about 29.8 mph. Thus the photo's
speed readout does not justify selecting a single exact frame from the visual
candidate range. Preserve this limitation rather than asserting +40 s or an
exact +52 s occurrence. The entire candidate window retains the previously
verified zero known fault fields, valid payload CRC/counters, unchanged SPI
error count and absent reuse diagnostics. Multiple visible popups do not
identify their source among still-undecoded or ECU/cluster-internal paths.

Private frame samples, sequential contact sheet, timing verification and
reproduction: `.analysis/archive/2026-09-30/ioniq5-photo32/`. No source media
or photo was committed, and no runtime/diagnostic behavior was changed.

## Cadence comparison with the user's vehicle

The user correctly emphasized that buffered reuse exists to preserve outgoing
cadence when a new host command is late. The buffered path consumes a command
or reuses its last body on each original incoming frame, substitutes that
incoming frame's counter and recomputes CRC. It is not an independent periodic
transmitter when original RX itself stops. Host sendcan jitter must not be
treated as an equal gap on the vehicle bus. Reuse exhaustion and the separate
direct cluster-message path require their own evidence.

Compared full rlogs from the user's Ioniq 5 PE `07b62e389ed26c81`,
`00000ff1--64803a8344--2` on clean `250f14ed`, and the reporting vehicle's
`000005aa--eadc607bce--46` on clean `db4aed1e` and ca/32 on clean `a315e806`.
The older 5aa upload is dated September 25; its warning status is unknown.
The user's report of no cluster warning in their car is contextual evidence,
not synchronized proof that the selected ff1 segment was warning-free.

All three record Ioniq 5 PE, flags 16787977, Hyundai CAN-FD safetyParam 157,
angle steering, Openpilot longitudinal control and alternativeExperience 1.
There are no carFw entries to compare ECU firmware versions. The Hyundai
vehicle source tree and safety_hyundai_canfd.h are byte-identical between
the user's 250f14ed and the incident a315e806 revisions. Reviewing September
18-30 changes found no change to the nominal rates of these buffered controls
or five direct cluster streams, nor the buffered queue/reuse limits. Changes
to payload generation are distinct from cadence changes.

| Observation | Older reporting vehicle 5aa/46 | Reporting vehicle ca/32 | User's vehicle ff1/2 |
| --- | ---: | ---: | ---: |
| Direct cluster host rate | 19.995 Hz | 20.000 Hz | 19.991 Hz |
| Maximum host cluster interval | 57.984 ms | 57.983 ms | 67.161 ms |
| Maximum returned cluster log interval | 65.796 ms | 64.953 ms | 73.579 ms |
| Maximum host-to-return delay | 17.284 ms | 16.067 ms | 29.818 ms |
| Unmatched direct cluster host packets | 0 | 0 | 0 |
| SPI checksum count, minimum/maximum | 1/1 | 67/67 | 1/9 |
| Reuse diagnostic reports | 8 | 0 | 8 |

The five direct streams are 0x161, 0x162, 0x1e0, 0x1ea and 0x200; all
their host packets have byte-identical returned echoes within 100 ms.
Their receive/return intervals are host log timestamps, not physical TX
timestamps. In particular, ff1's 73.579 ms returned interval cannot establish
expiration of Panda's 70 ms direct-forward blocking timer.

The user's ff1 segment includes reuse of MDPS, angle/LFA, SCC and TCS, with
no reported exhaustion. Returned counters for those buffered controls are
continuous, and all selected CAN-FD payload CRC checks pass. Its direct
CCNC_0x162 counter has 27 repeats and 27 +2 steps, confirming that this
previously observed counter-copy behavior is not unique to the reporting car.
The reporting vehicle's older 5aa segment already contains buffered reuse;
it is not a newly introduced behavior in ca/32. Its old host MDPS/TCS copied
counters repeat/skip, while buffered returned counters remain consecutive.

These comparisons do not support attributing the warning to reuse itself,
a recently changed nominal send rate, or greater host-log jitter in ca/32.
They do not prove ECU acceptance, identical regional firmware/configuration,
or absence of an earlier trigger. Returned echoes establish FIFO submission,
not successful bus ACK or receiving-ECU acceptance. A passive measurement of
actual TX completion / receiving-side traffic together with synchronized
warning occurrence would distinguish a physical delivery problem from an
ECU payload/state-consistency rejection. No cadence, reuse limit, warning
suppression or runtime behavior was changed.

Private reproduction and compact results:
`.analysis/archive/2026-09-30/ioniq5-cadence-comparison/`.
