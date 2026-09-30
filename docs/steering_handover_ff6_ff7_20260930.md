# Ioniq 5 PE handover trial: delayed recovery after force release

The user reported a delayed tap/catch when abruptly releasing steering force while
trying the live handover modes. Three uploaded segments on clean `b8a8a532ab` were
examined: `ff6` segments 2/3 and `ff7` segment 1. This is analysis of the implemented
experiment, not validation of its physical safety or a new controller change.

## Setting and data verification

`initData` repeats the startup parameter snapshot and is not the live mode record.
Although ff6's startup snapshot says 1, the controller's diagnostic messages show
mode **2** at the beginning of segment 2 and a live change to **3** at 17.775 s.
Segment 3 remains 3; ff7 segment 1 also uses 3. Runtime switching therefore worked.

The full cereal schema contains 10,288 carState messages across ff6 and 5,999 in ff7.
No carState CAN invalidity, reported steering fault or invalid model publication
occurs in these supplied windows. Maximum observed model publication age is
62/66 ms, and maximum carState publication gaps are 26.7/22.0 ms respectively.
These are publication observations, not proof of every internal execution time.

The original authority block and experimental helper were replayed on recorded
inputs. ff7's selected output matches within 0.017 CAN authority units. In ff6,
99% of active checked frames match within 0.000004 units; 14 frames differ by over
one unit around a later re-engagement near segment-3 38.9 s. Reconstructing the
latest CC at carState publication is not identical to card's earlier subscription
snapshot at that engagement edge. Do not claim bit-exact whole-route replay.
The release windows below are separately supported by transmitted CAN ceilings.

## Release timing

Times are relative to the first carState in the named segment. "Release" below
means the torque-based `steeringPressed` falling edge, not a measured hand-removal
instant. The ceiling is a CAN authority allowance, not actual physical motor torque.

| Segment / release | Ceiling starts rising | Ceiling reaches 250 |
|---|---|---|
| ff6-2 / 5.488 s, mode 2 | 5.838 s: 350 ms later | 6.332 s: 843 ms later |
| ff6-2 / 13.001 s, mode 2 | 13.321 s: 320 ms later | 13.862 s: 861 ms later |
| ff7-1 / 34.081 s, mode 3 | 34.606 s: 525 ms later | 35.091 s: 1.010 s later |
| ff7-1 / 36.053 s, mode 3 | 36.545 s: 492 ms later | 37.533 s: 1.480 s later |
| ff7-1 / 50.714 s, mode 3 | 51.234 s: 520 ms later | 54.229 s: 3.515 s later |

At ff6-2 5.488 s, actual angle is -28.8 degrees and target is -29.2 degrees:
only 0.4 degrees apart. By 5.698 s actual angle has unwound to -16.6 while the
target is -27.45; authority is still 25. The rapid-release detector fires in replay
at this point, but error is now 10.85 degrees, exceeding its two-degree gate.
Normal recovery begins at 5.838 s with an 11.81-degree error. EPS-reported output
then grows in magnitude and actual steering returns toward the target. The ceiling
ramps rather than jumping straight to 250; delayed recapture of accumulated angle
error is a plausible source of the reported catch/tap, not confirmed tactile timing.

The 13.001 s release similarly produces a rapid candidate only at 13.341 s, with
12.86 degrees of error. Force oscillating across the low threshold repeatedly
restarts the confirmation interval. Normal recovery has already begun by then.

## Why the experimental path missed these releases

The implementation first demands raw and median effort <=0.3, then 100 ms of
continuous low force, and finally a small current steering error. These requirements
can become mutually unhelpful during release: while the controller waits at a low
ceiling, actual steering moves away from the desired angle, so the later small-error
gate refuses recovery. In the first example, even the first raw <=75 torque sample
at 5.581 s already has more than four degrees of angle error. Merely shortening the
100 ms confirmation would not solve every observed case.

Other releases miss the separate 250 ms strong-to-low deadline or the 300 ms
sustained-override arming requirement. In ff7's final example the last strong sample
is at 50.710 s; low raw force first appears at 51.069 s and low raw+median at
51.075 s, already outside the deadline. No rapid recovery is selected. Force at
the column is not a direct measurement of a hand-removal gesture.

The existing repeated-override logic then lengthens normal recovery from 0.5 to
1, 2 and eventually 3 seconds. This remains active in all experimental modes.
ff7's final case reaches repeat count 3: after a 520 ms wait, it takes another
three seconds to ramp from 25 to 250. During the wait, the angle changes from
28.4 to 21.0 degrees while the target remains near 30 degrees.

## Separate short offer and withdrawal

The helper never selects rapid recovery in the reconstructed supplied windows.
ff7 has one short mode-1 offer within mode 3: 49.885–50.078 s. The recorded ceiling
increases from 25 to 46.057, then returns to 25 in about 12 ms. Actual/target error
is only 1.21 degrees at withdrawal, but its filtered growth rate exceeds the
2 degrees/s abort threshold. Driver effort is decreasing, not increasing or
reversing. CAN confirms the 46→25 change. This is a second possible tactile
discontinuity, distinct from the later release and legacy recovery.

## Visual and sensor context, next design direction

Road frames were aligned by qRoadEncodeIdx.segmentId and timestampSof, not arbitrary
video-player seeking. The two ff6 examples occur on a curved rural road; ff7's
final example is on a gentle bend approaching a marked crossing. Video supports
the road context; it does not show the driver's hands or measure the tap.

Original steering-touch state is present and valid. Reported contact loss comes
later than torque release in these examples: 5.812/13.221 s in ff6 and 52.117 s in
ff7. It therefore is not demonstrated to provide the earlier release cue needed
here; contact-state edges are not hand-removal ground truth.

These logs support revisiting the handover design rather than simply raising its
maximum torque or removing the angle-error gate. A candidate should use the recent
pre-release alignment and force trend, start with a limited takeover referenced to
current steering, and blend toward model steering with bounded command/authority
changes. Growing error caused by release must be distinguished from renewed driver
opposition before choosing withdrawal. Such a change needs separate closed-loop
and vehicle testing; replay of unchanged inputs cannot establish its effect.

Private traces, diagnostic messages, contact sheets and reproduction scripts are
indexed in `.analysis/archive/2026-09-30/ff6-ff7-handover/`. No captured images or
raw logs are committed. Controller behavior remains at the tested revision.
