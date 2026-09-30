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
