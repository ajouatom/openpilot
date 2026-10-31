# Turn indicator tracking and cancellation review — 2026-10-31

The owner requests reliable audioTurn-to-completion indication, automatic cancellation
when driving straight past a planned turn, and review of manual model-path departure
and front-radar admission. This change affects the comma indicator source only.

## Adopted behavior

- Start a fresh audioTurn cue at a full stop in Drive with continued lateral permission,
  no steering fault and the existing fresh vehicle/model/navigation guards. Moving
  new starts and all lane-change indications retain actual lateral activation.
- A receiver-accepted send followed by 1.5 seconds of both lamps off can use the
  existing single recovery attempt even if initial illumination was never confirmed.
  Initial retry additionally requires receiver ready. It never retries a rejected send,
  manual/driver cancellation, loss of acknowledgement or communication failure.
  One total recovery attempt per turn; original 300s expiry and four-second per-attempt
  activation expiry remain. CAN schedule, counter, receiver and hold packets unchanged.
- Keep a fresh opposite audioTurn cue for at most3s while the previous physical turn
  completes. No reversal mid-turn. Stale, mode/input reset, navigation change and
  driver commands discard pending cues; policy suppression still dominates.
- Optionally bind the current TBT point to the cue, using matching current guidance
  distance and a fresh external-navigation session. Use raw comma GPS only, never
  phone location or model-map route projection. Missing coordinates retain the
  existing physical/heading/navigation-distance completion policy.
- Location binding requires an ahead point8..150m, Euclidean/TBT distance agreement
  within25m, approach bearing within25deg and GPS horizontal accuracy>0 and<=8m.
  GPS age<1.5s, instruction source age<2s; timestamp duplicates are not new motion.
  Discontinuities/gaps reset passage evidence. A closely approached point<=15m then
  radial separation of15m supplements physical completion; bound fresh GPS still ahead
  prevents early completion even if the TBT text/distance advances.
- Straight cancellation requires close approach<=15m, passage beyond20m+2*accuracy
  along the incoming direction, cross-track<=10m, heading difference<15deg, wheel
  angle<=10deg, speed>=2m/s and1s confirmation from new GPS fixes; no completed turn.
  It cancels after passage, not by predicting driver intent before the intersection.
  Fresh explicit off-route/guidance-ended status also cancels. No same-cue replay.
  A repeated cue for the same cancelled point/direction/session is suppressed for60s;
  a different target or navigation session does not inherit that suppression.
- Existing bounded physicalBlinkerState diagnostics add phase/position-availability
  only; no coordinates or new USB fields. This records inference, not visual acceptance.

## Reviewed, not changed

Manual indicator remains driver torque plus lane-boundary geometry, ATC independent,
lamp-only,3s retry/1s extinction and12s own-feedback isolation. Requiring departure
from the model path can miss maneuvers because the predicted path can follow the
same steering direction; it is not a fixed reference road center. Do not add this
mandatory gate or relax boundary thresholds without detection replay evidence.

Ordinary ATC already uses side-target radar distance/relative speed and BSD through
SideState. The separate250m add-on checks the full right-front list with current and
3s projected gap>=5m. These remain unchanged. Neither no radar detections nor a clear
lead identifies the correct exit; radar is not a substitute for lane/route geometry.
No new automatic steering or lane-change admission is introduced.

## Verification and limits

Desktop indicator/source/helper suites695 passed,26 native safety cases excluded;
location/source/schema cases28 passed. Includes GPS jumps, stale/missing fixes,
parallel roads, stopped vehicles, real turns, pending opposite cue expiry, schema
roundtrip, initial recovery limits and cancellation. Native CI follows publication.

Read-only replay of twelve previously hash-verified117f logs:14262 frames, zero usable
raw-GPS/TBT anchors for this new classifier, zero old/new turn-output differences.
This is fallback regression evidence, NOT location-cancellation validation. Among93
near-boundary driver-steering samples,15 model paths already point toward the driver's
steering direction at10m; samples are not independent lane-change events. Private
results remain under .analysis; no location or device identifiers published.

No live vehicle write/reboot/CAN actuation or physical/road validation. Git Pull and
comma restart deliver this source update; Jetson runtime/model/bundle unchanged.
Sender-provided TBT coordinates and reliable vehicle GPS are prerequisites for the
new location-based behavior. No guarantee of reliable lamp acceptance or correct
junction classification is made from synthetic tests.
