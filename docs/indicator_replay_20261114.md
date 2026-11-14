# B1 hold and indicator replay, November 14

## Recorded evidence

Six Ioniq 5 PE drives from November 13–14 use recorded code `162da468cc`.
The 60 full rlogs total 803,863,408 bytes and passed remote/local hash comparison.
The observer replay processes 68,873 frames after initial input alignment.

Of 33 recorded manual-turn observations, 31 have a nearby physical lamp onset.
All 31 lamp episodes last about 2.48–2.50 seconds, although several observations
continue much longer. The remaining two observations have no nearby B1 request.
Across all sources, 60 of 63 B1 requests report `driver_override` within
0.351–0.694 seconds. Receiver subreasons were absent from these logs.

The release-4 receiver accepts its injected 0x413 lever echo only while the
same-side lamp is lit and the tap window remains open. The captured echo persists
through dark flash intervals. This can clear the acknowledgement required by the
Ioniq 5 PE continuous-hold path. A receiver source replay supports this failure
mechanism. The running firmware identity and exact historical subreason remain
unverified.

## Changes

- B1 release 5 remembers a confirmed same-side echo first seen during the tap
  window. Its continuous report remains acceptable through dark intervals.
  Neutral feedback after the tap closes that allowance. A subsequent lever
  report cancels ownership. Independent raw 0x3C1 comparison, opposite lamps,
  invalid CAN, session/lease expiry and the ownership limit continue to cancel.
- The comma reads one bounded receiver diagnostic per failed request before
  STOP replaces the reason, recording `b1TapDiagnostic`.
- Manual boundary tracking tolerates a retreat lasting less than 150 ms while
  withholding new requests during that retreat. A bounded paired position
  change is accepted only during recent driver intent with consistent lane
  width and the existing adjacent-lane checks.
- A same-direction manual turn recognized during a manual lane indication
  inherits the existing request identity and follows turn completion conditions.
- Model-turn indication requires same-direction steering of at least 15 degrees
  and fresh measured rotation of at least 3 degrees/s. A withdrawn turn
  probability below 30%, moving wheel return within 10 degrees and 400 ms of
  stability can end a hold before the wheel has reached the older 30-degree peak.

## Fixed-input comparison

| Event | Before | After |
| --- | --- | --- |
| Nov 14, route 11d9, segment 2 + 44.458 s, right approach | Brief boundary retreat erased intent | Right manual request |
| Nov 14, route 11d9, segment 4 + 21.688 s, right approach | Paired boundary shift reset tracking | Right manual request |
| Nov 14, route 11d9, segment 5 + 30.346 s, frame 6609 | Left model-turn request on a straight road | Blocked by measured-motion conditions |
| Nov 13, route 11d4, segment 9 + 35.356 s | Manual lane tracking | Same request continues as a manual turn |
| Nov 13, route 11d5, segment 0 + 27.791 s | Manual lane tracking | Same request continues as a manual turn |

The comparison has 61 original and 62 revised requests. The other 60 requests
match in direction and start time. The isolated native B1 feedback replay covers
62 captured tap starts and 1,820 subsequent 0x413 frames. The old predicate
rejects every sequence, while the new predicate accepts their own-tap feedback.
One of the 63 requests lacks an identifiable captured tap start and is excluded.

The model false indication had left-turn probability 0.501, wheel angle −1.6
degrees, near-zero rotation, no driver steering and no ATC turn request. Its
original observer lasted about 30 seconds. Camera review shows straight travel.

## Validation and limits

Eleven focused regressions fail against the recorded baseline. The revised
local indicator, Bluetooth and update suites pass 878 checks, with two
platform-specific skips. The native C++ feedback tests pass separately under
MSVC. ESP32-S3 compilation succeeds with Arduino-ESP32 3.3.5. Wiki generator
tests pass 25 checks.

The bundled application is release 5, 651,488 bytes, SHA256
`bf60f950cf3f1334fd9e4fe932a55b45a1e2cf2a2d521207196a6884b79a3b4b`.
Its ELF identity is
`eb1e8753c52398cded38e186395b4a17d61f22b760cf9ce000af376027b2b39f`.
Normal Git Pull and a full comma restart deliver it through the existing
authenticated B1 automatic updater when the indicator is idle.

Observer replay retains recorded vehicle motion and lamp/stalk inputs and
supplies bounded own-tap context. It cannot simulate changed physical lamps,
receiver scheduling or changed vehicle motion. The B1 predicate replay assumes
confirmed tap delivery. Native counterexamples cover opposite lamps, late first
echo, neutral-then-new lever input and unconfirmed transmission. Physical
ten-second hold and cancellation need a subsequent vehicle test.

Retaining steering evidence below 5 km/h was also tested. It added a parking
indication that suppressed an existing subsequent left turn, so that experiment
was reverted. A maneuver already strongly steered before reaching the minimum
speed can still be missed. These six drives do not establish a general
false-positive rate.

Private logs, video frames, replay scripts and outputs are indexed in
`.analysis/archive/2026-11-14/indicators/INDEX.md`.
