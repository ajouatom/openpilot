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


## November 15 follow-up after vehicle deployment

The November 14 evening and November 15 morning drives run `91eb1e86bb`.
The B1 diagnostic records identify release 5 and the ELF listed above. These
two drives contain 35 B1 requests and zero reported `driver_override` events.
Three requests report `twai_alert` with `0x600`, combining bus error and failed
transmission. One of those confirms zero transmitted frames and has no lamp
onset. Two other moving turns lose the accepted request ID while the receiver
still reports `waiting`. Their receiver subreasons were not captured.

Camera review and recorded inputs identify additional software defects.

- A junction boundary relabel suppressed manual-turn admission for two seconds
  above 30 km/h. Steering had reached its peak when detection resumed, losing
  the required angle growth. Manual turns now retain their independent torque,
  angle-growth, rotation and curve checks throughout boundary relabels.
- A jump of only the far boundary erased a continuous manual approach. A recent
  established driver approach now waits for a valid sample for up to 150 ms when
  the target boundary remains continuous. Requests pause during that gap.
- Current model lane-change probability could trigger with little lateral
  movement. It now also requires both measured boundaries to move at least
  0.15 m toward the crossing over 0.30–0.55 s, within 1.5 m of the target line.
- At two stops, very small negative speed estimates ended an owned manual-turn
  hold. Drive plus confirmed standstill now permits estimates within 0.1 m/s
  of zero during an existing hold.
- Loss of a previously accepted B1 request now captures the receiver diagnostic
  once even while its state remains `waiting`. Ownership transitions remain
  logged through the hold lifetime. This supplies evidence for the two unresolved
  moving-turn interruptions.

The combined fixed-input comparison covers 92,836 model frames across eight
routes and 81 full rlogs, including the previous six drives. Baseline produces
99 requests and the candidate produces 102. Of the baseline requests, 97 retain
exact direction and onset. One manual turn starts 0.447 s earlier and one
camera-reviewed straight-road model indication is suppressed. Four manual
requests are added.

| Segment and offset | Replay result |
| --- | --- |
| 11e1/0 + 55.400 s | Previously missed junction left turn detected |
| 11e0/98 + 40.077 s | Previously missed left lane approach detected |
| 11e1/2 + 0.757 s | Straight-road left model indication suppressed |
| 11e0/98 + 22.282 s | Existing left turn detected 0.447 s earlier |
| 11d5/1 + 13.250 s and 11d9/6 + 50.390 s | Additional right manual requests in older logs |

The two added older requests have recorded driver and boundary evidence, but
retained camera video was unavailable for their new visual classification.
The comparison preserves recorded lamp feedback and vehicle motion and supplies
bounded tap-echo context. Changes to real lamps could alter later admission.
The standstill correction also passes the source-loop test with exact-request
acknowledgement and lamp feedback.

Seven focused regression cases fail before these corrections. The indicator,
Bluetooth and B1 suites pass 898 local checks, with two platform skips. The three CAN transmit
faults and the two moving ACK-loss causes remain unresolved by this patch.
Vehicle installation and physical verification of these November 15 changes
remain pending. Private evidence is in
`.analysis/archive/2026-11-15/indicators/INDEX.md`.

## November 15 complete retained-log follow-up

This follow-up uses baseline `b12855935aa` and 403 identified full-log segments
from 52 routes. There are 407 local full-log copies and 181 additional qlog-only
segments. The aligned observer replay covers 452,218 frames. It reconstructs
pre-inference carState/carControl samples, uses recorded actuator curvature and
the previous lane-change phase, and supplies bounded same-request tap echo.
All observer options are enabled for the comparison. Exact process receipt times,
receiver state and changed physical lamp feedback are not fully reconstructed.
Seventeen files have an incomplete tail, for which only complete events are read.

### Implemented corrections

- TBT travel accumulation treats a small negative filtered speed as zero when
  the vehicle reports standstill in drive. Three retained stopped episodes
  previously invalidated the journey at −0.00096 to −0.00303 km/h. Reverse,
  invalid inputs, moving negative speeds and values at or below −0.1 m/s retain
  their invalidation behavior.
- B1 renews an authenticated, acknowledged hold with fresh source ownership
  before its next metadata read. A delayed read could consume the remaining
  lease before the former read-then-write sequence refreshed it. The command
  retains the original source expiry and the receiver's local guards. Fresh
  release or override after the read supersedes the renewal. Bounded exchange
  timing and the last eight exchanges accompany failure diagnostics.
- Manual turn angle growth follows a driver's partial unwind and renewed
  same-direction steering within the current intent. Motion before driver
  admission is excluded from this additional evidence.
- Road-curve recognition reads individual cereal list elements. Cap'n Proto
  list slicing raised TypeError and silently disabled the original curve test.
  New tests use serialized production-schema messages.

### Before and after

| Check | Baseline | Revised |
| --- | --- | --- |
| Observer requests across 52 routes | 298 | 299 |
| Existing requests removed | — | 0 |
| Existing request starts | — | 296 exact, two about 0.10 s earlier |
| Existing request ends and promotions | Recorded-input baseline | Exact agreement |
| Missed right turn, 11e3/13 | No request | 53.048–59.288 s |
| TBT journey starts | 61 | 61 |
| TBT normal completions | 55 | 58 |
| TBT input-invalid endings | 4 | 1 |
| TBT journeys clipped by retained-log end | 2 | 2 |
| Injected 200 ms metadata delay | Acknowledged lease expires | Renewal arrives before expiry |
| Production cereal curve regression, both directions | Fails | Passes |

The TBT component assumes mode 2 and does not reconstruct full source admission,
navigation target identity or receiver delivery. Only the three stopped endings
change. The remaining input-invalid ending is at a segment boundary where the
scan selected carState/carControl timestamps 6–10 ms newer than the model row.
It is a replay alignment limitation and does not establish a fourth vehicle fault.
The added right turn is visually confirmed. Its detection still occurs after
the turn has begun. Two earlier starts occur at 11e1/0 and 11e5/4. Every existing
observer end and promotion remains at the same recorded time.

The 889 indicator tests and 77 Bluetooth tests pass locally, with eight platform
skips. Five new behavioral regressions fail against the isolated baseline.
Three recorded negative-speed inputs independently reproduce the old invalidation.
Focused undefined-name and syntax checks pass. Full-file Ruff reports existing
legacy style/import findings in the touched test files and the B1 owner import.

### Remaining observed failures

Four visually classified unmarked-road curve indications remain in 11e3/15,
11e3/16 and 11e4/2. A road-edge fallback removed genuine fork indications as well
and was rejected. A broader pre-intent steering history also introduced an
unwanted left indication in 11d6/0 before a right entrance and was rejected.
The final replay adds only the confirmed 11e3/13 right turn.

Two observations near the start of 11e5/0 have no corresponding B1 request or
physical lamp. Historical source admission reasons were not recorded. Four
release-5 receiver failures explicitly report lease expiry, but their ATT
read/write delays were not logged. The injected delay test establishes a repaired
scheduling failure mode. It cannot establish the cause or cure of each of those
four historical events. New timing diagnostics support that distinction.

In the latest three recorded drives, 50 of 52 observations have physical lamp
onset, with 189 ms median and 256 ms P95 detection-to-lamp latency. Those are
baseline measurements. Revised physical latency and uninterrupted real hold
remain unverified. The baseline aligned replay matches 51 of those 52 starts,
with one feedback-dependent cancellation mismatch affecting the subsequent start.
No vehicle write or CAN actuation is part of this follow-up.

Private before/after evidence and all-route tables are indexed in
`.analysis/archive/2026-11-15/indicator-fixes/INDEX.md`.
