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

## Changes

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
