# Ioniq 5 PE 5d3-21 steering investigation

The reporter clarified: "First 10 seconds held too far inside of curve. Then,
after 35, didn't turn well when hitting end of street." The first complaint is
inside-curve positioning; the second is insufficient turning at the street end.
The initial interpretation as a vibration/catching complaint is superseded.
Column-torque oscillation remains a separate measured observation, not the
reported symptom or a demonstrated cause of the first complaint. This is an
investigation, not a controller change or a physical root-cause diagnosis.

## Evidence and time reference

- Vehicle: HYUNDAI_IONIQ_5_PE, dongle `7f419ed56030e135`.
- Segment: `000005d3--54fb087f33--21`, clean `eb58e6d7b04e`.
- Original NAS `rlog.zst` and `qcamera.ts`; full cereal schema: 5,999
  carState/carOutput, 6,009 carControl, and 1,200 modelV2 messages.
- Times below start at the first carState, monotonic 1346.494976932 s.
  Video frames are joined using qRoadEncodeIdx.segmentId and timestampSof.
- Startup SteerHandoverMode=3 is corroborated by runtime diagnostics throughout
  the segment. AlwaysLateral=1; latActive remains true throughout both reported
  windows, including after pedalPressed/userDisable longitudinal disengagements.
- Current Hyundai controller/helper sources match the incident commit. This
  vehicle already has the September 30 torque-ceiling-only handover revision.

The 25/250 values below are CAN authority ceilings, not measured motor torque or
physical percentages of assistance. Column torque is quoted in its decoded signal
units; it must not be labelled Nm or equated directly to EPS output torque.

## First 0–10 seconds: reported inside-curve positioning

The video shows a right bend, with speed decreasing from a peak of 57.3 to
30.5 km/h. Actual wheel angle reaches -54.8 degrees. The target and limited command
are close to actual angle: maximum sampled target/actual error is 3.76 degrees.

The clarification shifts the question from whether the EPS follows its command
to whether that command produces the desired road position. At 5.51 s, actual
angle is -54.8 degrees and target is -54.6: close agreement does not establish a
correct path. During entry (for example 4.01 s: actual -20.8, target -24.2), the
target requests more rightward steering than the actual wheel has reached.
During exit (8.51 s: actual -28.9, target -26.0), actual steering remains farther
right than the unwinding target, which can contribute to staying inside longer.

This supports examining desired path/curvature, curvature-to-angle conversion
and response/unwinding timing. It does not isolate a driving-model path error:
no calibrated lane-relative lateral-position reconstruction has been performed,
and the reporter has not identified the precise instant within these ten seconds.
Small steering-angle error alone cannot quantify the road-position error or
exclude a meaningful contribution from tracking delay.

The authority ceiling is **250 throughout**, and steeringPressed is false on all
1,001 sampled frames. Nevertheless, the column-torque signal alternates rapidly,
spanning -236 to +335. During 3.8–5.5 s, its dominant 4–20 Hz spectral peak is
approximately **9.94 Hz**. The reporter did not identify vibration as the symptom;
do not substitute this signal observation for the clarified positioning complaint.

The approximately 10 Hz signal is present in original bus-0 MDPS 0xEA reception;
it is not created by plotting carState or by the modified camera-bound MDPS copy.
The commanded angle progresses through the bend without a comparable large
alternating angle command. Smaller actual-angle and EPS-output variations are
also present. This does not establish whether the source is EPS response, steering
mechanics/road excitation, or an interaction with the driver and angle controller.

Rapid-release recovery cannot explain the onset here: it has not activated, and
the ceiling never drops. Changing only the handover mode would leave these
full-authority outputs unchanged on the same recorded inputs; it is not a
demonstrated remedy for inside-curve positioning.

## 35 seconds onward: large turn, angle limit and repeated yielding

The video shows the street-end left turn alongside a retaining wall, at roughly
19–24 km/h. The clarified insufficient-turning complaint is supported by both
command clipping and subsequent large actual/command disagreement. The observed
sequence is:

| Time | Recorded behavior |
|---|---|
| 33.5–36.35 s | Column torque spans -542 to +415 with a dominant peak near 10.14 Hz. No steeringPressed yet; authority is 240–250, mostly 250. |
| 36.19–37.09 s | Host angle command saturates at the existing **175-degree software limit**; upstream target peaks at 190.28 degrees. Panda TX echoes show the same plateau at 36.21–37.11 s. |
| 36.402 s | steeringPressed first becomes true in this turn: wheel 175.7 degrees, target 184.6, limited command 175. Authority starts falling from 250. |
| About 36.6 s | Authority reaches 25. Lateral control and the CAN angle-active field remain active; this is reduced assistance, not a full lateral disengagement. |
| 37.10 s | Wheel has unwound to 105.5 degrees versus target 173.5, while authority has only recovered to about 46. |
| 37.58 / 38.15 / 38.63 s | Renewed pressed detections interrupt recovery again. Around 38.13 s authority is 156; another withdrawal brings it to 25. |
| **39.125 s** | Wheel **51.7 degrees**, target/limited command **147.1 degrees**: a **95.4-degree steering-wheel error**, at 23.7 km/h. |
| 39.28–39.4 s | Column signal reaches +839 and wheel rapidly winds back toward the turn; another pressed detection lowers authority again. The relative contributions of EPS recapture and deliberate driver correction are not identifiable from these signals alone. |

The causal code path is clear for authority withdrawal: CAN-FD steeringPressed
requires abs(column torque)>250 for six consecutive carState updates; sign is
not part of that condition. Once pressed is true, the legacy ceiling loses
20 units per control tick. The runtime mode remains 3, but diagnostics show
`waiting` throughout 35–39 s: the large drops here come from the legacy override
path, not repeated experimental mode-1 offers.

The earlier alternating torque evolves into longer negative-force intervals around
36.4 s, meeting that criterion. These are valid detections under the implemented
rule; the log cannot establish that each represents intentional driver takeover.
Original touch reports no contact throughout 0–10 s and nearly all of 33–36.4 s,
then reports intermittent contact. Touch is supporting evidence, not ground truth
of hand position or intent.

The 175-degree limit contributes to the initial command/target mismatch but cannot
alone explain the later 95-degree gap: by 39.13 s the requested 147 degrees is
inside the limit and is actually transmitted. Repeated low authority coincides
with the wheel unwinding, then partial recapture. This directly supports a
failure to maintain the requested turn. Whether the requested path was itself
adequate for the street-end geometry remains a separate, unresolved question.

## Experimental recovery still has a discontinuity near turn exit

Runtime diagnostics show capture at 40.340 s, recovery at 40.401 s, and blocked
at 40.489 s. The selected ceiling reaches 44.33, then drops to 25 when raw column
force crosses the recovery veto of 0.8*250=200, despite steeringPressed remaining
false. Original driver readings reach -206 and -250 around this edge. The log
does not distinguish renewed driver opposition from steering response force.

Panda TX echoes independently confirm **44 to 25 in 7.1 ms**, at
40.494–40.501 s. This is an observed output discontinuity, not just a replay.
The helper enters returning at 40.743 s and limits the rise to 110 units/s,
reaching 250 around 42.79 s. Legacy authority was already 250 by the 41.374 s
diagnostic; mode 3 therefore prolongs reduced authority in this exit window.

Unlike the earlier ff6/ff7 implementation, this revision does start early capture.
It still withdraws abruptly on the raw-force veto and then returns slowly. No
mode-1 offering state is recorded in this segment. Model yStd[10] exceeds 0.3
briefly near 37.4 and 39.3 s, invalidating experimental evidence and slowing the
legacy uncertainty-dependent recovery; this is uncertainty, not a missing or
invalid model publication.

## Checks and limits

- No carState CAN invalidity or temporary/permanent steering fault. All 6,000
  original MDPS frames have decoded LKA_FAULT=LFA2_FAULT=0.
- All 1,200 model and livePose publications are valid. Maximum observed model
  publication age is 63.0 ms; road-camera SOF maximum gap is 54.49 ms. No recorded
  stream failure explains the reported windows.
- Panda safetyTxBlocked and safetyRxInvalid remain zero; SPI checksum error count
  stays at the pre-existing value 76. No new increment is observed.
- All 1,200 original and transmitted 0x162 VIBRATE fields are zero. carControl
  requests no lane-departure or visual alert. The decoded warning-vibration path
  does not explain the oscillation; this does not decode every EPS internal mode.
- Runtime steerSaturated events occur briefly around 4.2 and 34.5–35.2 s, but no
  steerSaturated alert is published. Such an event is not proof of a hardware fault.
- Unchanged-input mode comparisons cannot establish what the driver/EPS would do
  with another ceiling, angle limit or filter. Do not raise the angle limit or
  weaken driver-override detection based solely on this recording.

With the clarification, the next investigation should separate first-curve road
position and target/unwinding timing from the street-end angle-limit and
override/recovery behavior. Improving only release timing leaves the first
symptom unexplained. Torque oscillation deserves separate investigation but is
not established as the positioning cause. Physical root-cause isolation needs
controlled comparison; the available recording establishes the signal and output
sequence, not a validated remedy. No controller or vehicle setting was changed.

Private reproduction scripts, CSV traces, spectrum methodology, contact sheets
and plots are indexed in `.analysis/archive/2026-10-01/ioniq5-5d3-steering/`.
