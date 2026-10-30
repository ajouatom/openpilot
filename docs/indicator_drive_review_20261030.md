# Indicator drive review — 2026-10-30

The owner reported a missing 250 m right indicator, a missing left turn lamp,
and manual lane-change indication that appeared coupled to ATC. Twelve closed
full rlogs were read from the owner's comma after checking the NAS first; all
162,453,945 bytes were checked against remote SHA-256 hashes. Device identifiers,
raw logs, extracted CAN and scripts remain private under `.analysis`.

## Recorded evidence

- The drive used `cb724a0cb8`, ATC mode 1, physical indicators mode 2,
  the 250 m add-on and manual-boundary option enabled, SPAS disabled.
- Segment 10, 18.003 s: right-fork TBT code 4, approximately 284 m remaining.
  The add-on admitted only right-turn code 2. It was also still outside 250 m.
  At 19.016–22.916 s both received camera CAN rear slots reported detected
  state 1 and distance 25.0 m; the OEM right BSD bits were clear. The existing
  rear validator treats this special value as unknown, not a measured safe gap.
  Right lane change starts at 21.554 s with driver torque; lamp feedback appears
  at 21.809 s. This is not evidence of a persistent lockout after ordinary ATC.
- Segment 8: fresh left audio cue at 41.909 s, source request at 41.911 s,
  receiver acceptance by 42.859 s, but no lamp activation. The unconfirmed hold
  expires at 45.952 s. In `blink_can.Controller`, acceptance is set after the
  scheduled socket sends complete; it does not establish vehicle acceptance.
  At 51.334 s and 58.072 s the recorded stalk count increments, followed by
  actual left lamps. A new cue at 55.553 s occurs while lateral control is
  inactive and the driver's previous signal is still present. No new automatic
  hold is recorded. The failed earlier physical activation remains unresolved;
  this change does not retry unconfirmed CAN taps or bypass driver priority.
- All 14,262 recorded model frames have no manual-boundary request. Removing
  the navigation busy condition alone still produces no detector candidates:
  confidence drops before the boundary reaches the original threshold.

## Initial correction (32ed45b2ec)

- Allow right turn (2) and right fork (4), including matching ATC/fork right
  preparation. Keep the fixed 250 m boundary and rear/BSD checks unchanged.
- Before any 250 m request exists, steering clears the one-second confirmation
  rather than cancelling the entire TBT window. Intervention after an owned
  request still cancels the session and cannot replay it.
- Manual boundary detection no longer depends on ATC type, turn direction or
  model desire probabilities. An executing lane change remains reserved.
  The physical source requires fresh model/vehicle data, not fresh navigation,
  and preserves existing output ownership and receiver readiness.
- Track the crossed boundary for at most one second after its last probability
  above 0.5. The opposite boundary must move consistently but need not also
  exceed 0.5. Finite geometry, lane width, continuous frames, boundary-jump
  rejection, sustained signed driver torque, speed and lamp/lever gates remain.
  Gaps and invalid/disabled/busy states discard tracking history. No arbitrary
  low-confidence boundary can originate a request.
- Keep the existing ATC steering/audio logic and manual-indicator feedback
  isolation. No new CAN format, receiver, model or Jetson bundle.

## Validation and limits

Desktop focused suites: 499 passed, 26 native-safety cases excluded on Windows.
Tests exercise the actual source loop during ATC preparation and stale navigation,
the real helper's detector admission, recent-versus-expired confidence, physical
feedback isolation, both TBT types and the retained distance/rear/cancellation gates.

Open-loop detector replay of the 14,262 recorded model frames changes zero
candidates to two left candidates: segment 10 at 50.552 s and segment 11 at
10.908 s. These are detector outputs, not end-to-end lamp acceptance or a
closed-loop driving result. The reported segment 10 interval remains blocked
by its actual distance/rear/driver conditions even with fork support.

Delivery is normal `carrot-wip` Git Pull and comma restart. This task made no
vehicle writes, CAN transmissions, device setting changes or reboots. Physical
lamp behavior and the earlier accepted-but-unlit left request require further
vehicle evidence; no complete road cure is claimed.


## Owner-requested BSD-only follow-up

The owner subsequently requested OEM BSD alone for the 250 m add-on's rear
check. This supersedes the earlier distance-veto policy above: corner slots,
20 m thresholds and 24.9/25.0 m special distances no longer gate this add-on.
The existing CarState TBT fields now publish freshness and warning state from
OEM `BLINDSPOTS_REAR_CORNERS` directly, independently of corner availability.
Missing/stale/short BSD messages and missing decoded warning fields still block.
The ordinary DesireHelper side-object and maneuver checks remain unchanged.
This is a change of admission policy, not evidence that 25.0 m was a measured gap.


The owner then specified a 5 m front gap with a three-second relative-speed
projection (superseding the briefly proposed 50 m). The add-on checks the full
right-front radar list plus its representative lead, not just `leadRight`:
that representative deliberately excludes objects at or below 5 m. Require
both current distance >=5 m and distance + vRel*3 >=5 m for each active front
object. Negative-distance rear objects use the BSD policy instead. Missing or
nonfinite distance/speed, stale/invalid radarState, radar errors or a configured
unavailable radar block initiation. The freshness/projection test applies during
confirmation and while waiting to start, not as a new steering command during
an executing maneuver. This is a constant-relative-speed admission check, not
a guarantee against braking or lateral motion. Existing stronger side-object
checks still apply. The right-side list can conservatively include farther
right lanes; it is not a newly validated target-lane tracker.

Final desktop checks: 679 indicator/desire checks passed, 26 native-safety cases
excluded; 53 rear/BSD/schema checks passed separately. Actual source-loop tests
cover close/closing objects, stale/invalid/faulted radar and unavailable hardware.
The recorded owner profile has radar available and BSD enabled. Reconstructing
fresh BSD directly from received bus-2 CAN and using recorded radarState, the
reported segment 10 confirms at 19.559 s (250 m) and emits a right request at
20.603 s (228 m). Front object: 32.212 m, vRel +2.2 m/s, projected 38.812 m.
Recorded driver intervention at 21.554 s cancels it. This open-loop result does
not predict the vehicle trajectory or prove actual lamp acceptance.

## Manual indicator retry interval follow-up

The owner requested a three-second retry interval. Separate the detection cooldown
(3 seconds) from owned-lamp feedback isolation (12 seconds). A new request also
requires one continuous second of valid all-dark feedback and fresh driver torque
and boundary approach; elapsed time never replays the previous edge. Existing
vehicle, output ownership and executing-lane-change gates remain.

Quick opposite requests preserve both direction masks, including through source
and model restarts using the existing origin journal. A failed journal write
continues to forbid output. No USB2CAN packet, receiver or steering policy changes.
Six new regressions cover rearm timing, valid darkness, retained feedback masks,
restart and journal-write failure. Desktop suite: 685 passed, 26 native excluded.
No live vehicle writes or physical lamp validation in this follow-up. Delivery:
normal Git Pull and comma restart; no new Jetson runtime bundle.
