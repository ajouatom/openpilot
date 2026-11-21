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


## November 17 retained-log follow-up

The evening November 16 route and two morning November 17 routes add 23 full-log
segments and matching camera segments to the retained audit. The comparison now
covers 55 routes, 426 identified segments and 477,798 aligned observer inputs.
Complete messages from truncated historical tails are retained. The recent
11ef/2 tail is also incomplete. File hashes match the downloaded source bytes.

Three corrections are applied.

- Invalid lane width supplies no new admission evidence. An established approach
  keeps its last valid sample through the existing 150 ms continuity limit.
  Invalid samples reset adjacent-lane admission time, so intermittent invalid
  geometry cannot accumulate enough time to establish a new approach.
- A fresh manual crossing already admitted by the model uses that decision at the
  common output stage. The duplicate adjacency lookup is removed for this source.
  The source still validates observation identity, isolation, freshness, vehicle
  state, ownership, physical input and competing requests.
- Native Kick samples the caller's clock after reading the worker socket. A source
  publication arriving during the read can otherwise look future-dated and revoke
  the request. A bounded request-end diagnostic records freshness, continuation,
  opposite-lamp and readiness state once per ended request.

With the external-output minimum, the accepted detector changes requests from
324 to 325. All 324 previous starts, sides, sources and ends agree exactly. The
added left request at 11ee/6+41.508s corresponds to a visually confirmed manual
crossing. With the existing native minimum the comparison is 323 to 324, again
with all existing starts, sides, sources and ends exact. Nine deterministic regression cases reproduce on a31a59ca2d before
correction, including four bounded width gaps, three source transports with a
changed marking label, and initial/continuing native socket timestamp races.
The source-loop comparison covers 96 ordinary USB, BLE and Kick scenarios with
768 identical outgoing packet records. Changed-label recovery is tested through
the complete source loop separately.

Broader candidates were rejected during replay and video review. Accumulating
steering during rearm adds indications while avoiding parked cars. Keeping invalid
width samples as admission evidence adds an unconfirmed indication at 11ee/9.
Extending every native returned/relabelled observation to two seconds suppresses
five historical requests in the fixed-input native counterfactual and introduces
a different turn request. The normal-completion-only minimum remains in effect.
The two recent short native right outputs have two raw CAN lamp on-pulses each.

Unresolved cases include the unknown OEM left marking and model relabels at
11f0/5, the close left/right yard entry at 11f0/8, and additional intersection
geometry misses. The interrupted native handshake at 11ef/1 has no recorded worker
receive timing, so the socket race is a verified defect and a candidate cause,
not a proven diagnosis of that historical event. Its physical correction requires
new vehicle evidence. Startup admission at 11ee/0 is also unresolved.

The replay fixes recorded motion and supplies bounded synthetic receiver context.
It does not reproduce live SubMaster delivery, changed vehicle motion or physical
lamp response. No whole-corpus false-positive rate is claimed. Local focused
checks pass 1,249 cases with nine platform-specific skips. Native/full CI is
checked after publication. Private evidence is retained under
`.analysis/scratch/2026-11-17-indicator-fixes` and
`.analysis/scratch/2026-11-17-indicators`.

## November 18 boundary lifetime and consecutive-turn follow-up

The retained corpus now contains 467 full-log segments from 60 routes, including
41 additional November 17–18 segments. A/B comparisons process 523,934 aligned
model inputs per variant. Both sides use identical recorded motion, input validity,
prior indication journals and lamp/lever feedback. Receiver readiness is simulated.
The production observer sources are frozen independently for every candidate.

Only an existing manual lane indication gets the new boundary tolerance. An early
retreat very near the boundary can wait 150 ms for paired lane reassignment, using
the pre-retreat pair as its anchor. A separate early uncertain-boundary case requires
same-side driver torque and opposite-boundary progress; it must establish continued
progress within 250 ms and recover stable centered geometry within three seconds.
Opposite driver steering, invalid inputs and bounded geometry/continuity failures
still cancel. New lane admission, left marking and right road-edge gates are unchanged.

For consecutive turns, only opposite steering that ends a recognized manual turn
can seed retained intent. The extension requires at least 10 km/h, expires in three
seconds, and bridges only the last 250 ms of steering before rearm. Emission still
requires growing same-side driver steering, the existing two-second rearm and one
continuous dark second. Ordinary fresh-steering detection runs independently, so
expiry of retained intent cannot erase an otherwise valid turn. Ordinary manual-turn
admission remains 5–60 km/h. No automatic steering or transport changes are added.

| Fixed-input output assumption | Requests before / after | Lost previous requests |
| --- | --- | --- |
| Native first submission at request | 363 / 365 | 0 |
| Native first submission delayed 410 ms | 362 / 364 | 0 |
| External output, existing zero minimum | 364 / 366 | 0 |

All three comparisons add the same two visually confirmed right turns:
11d6/0+17.410s and 11f5/8+49.368s. Two existing turns start about 100–102 ms
earlier with exactly the same ends. Every other existing start/side/source is
unchanged. Five existing lane requests have changed ends; all other ends agree.
The following durations are observation requests, not measured physical lamp times.

| Request | Before | After, native with 410 ms submission delay |
| --- | --- | --- |
| 11f5/4+17.162s right | 0.200s, returned | 2.990s, completed |
| 11f5/7+28.690s left | 0.246s, returned | 2.446s, completed |
| 11d9/4+21.688s right | 0.156s, returned | 2.202s, relabel |
| 11e1/3+9.716s right | 0.103s, returned | 2.430s, completed |
| 118a/9+40.860s left | 0.203s, returned | 0.351s, relabel |

The last case remains shorter than the modeled 410 ms preparation delay and is
not counted as a recovered physical indication. The new 11f5/8 turn requests
1.700s before opposite driver steering cancels it; cancellation deliberately takes
priority over a two-second normal-completion minimum. Two full flashes are not
promised for this cancellation case. The other new turn requests 2.445s under the
delayed native assumption. No new false indication was identified in the two added
video scenes; this is not a whole-corpus false-positive rate. Existing falsely
classified curves/parking maneuvers are not comprehensively labeled or eliminated.

Broader candidates were rejected: retaining an entire steering sweep introduced
parking and exit-alignment indications and delayed subsequent requests; sharing
that memory with ordinary detection erased an existing turn when memory expired.
The retained 250 ms bridge and independent ordinary observer avoid those reproduced
regressions. Three deterministic failure-shape regressions fail on the baseline
and pass after correction. Local observer/source/native-ownership checks pass
1,184 cases with nine platform-specific skips; Linux CI separately covers native
imports and integration after publication.

Still unresolved: the 11f5/6 2.463 m lane-width admission rejection, unknown left
markings, and the 11f3/4 source-delivery gap without a recorded worker refusal.
Recorded output feedback cannot prove how a newly emitted lamp would affect a
closed-loop drive. No vehicle installation, CAN actuation or physical cure is
claimed. Private inputs, frozen sources, all comparison tables and visual evidence
are indexed in `.analysis/archive/2026-11-18/indicator-continuity-fixes/INDEX.md`.

## November 20: owner-requested manual admission and short-attempt rearm

Remove the OEM left-marking/centre-line and right-road-edge-clearance gates from
the manual boundary observer. Intent, speed, valid lane width, boundary movement,
confidence, driver cancellation and lamp-to-steering isolation remain. Model
desire/path admission and Tesla's explicitly supplied lane policy are unchanged.

A manual boundary attempt ending within two seconds through return or lane
reassignment may seed an independently confirmed same-side manual turn. It needs
same-side steering at termination, a fresh growing turn, valid yaw and at least
10 km/h within three seconds; driver/input cancellations cannot seed recovery.
A fresh opposite boundary approach may rearm after one continuous dark second
following a short abandoned attempt. Neither path revives the old request identity
or shortens the 12-second automatic-steering feedback protection.

The retained corpus remains 467 full segments, 60 routes and 523,934 aligned
inputs. Against 34ce0aa171, request counts change 365→406 for immediate native
submission, 364→405 with a simulated 410 ms first-send delay, and 366→407 for
external output. Each comparison adds 42 requests, loses one model-turn request
and changes 12 matched requests; 352 immediate-native observations retain exact
start/source/end. Matching reserves exact observations, then same-source matches,
then same-side matches within two seconds. Counts are not truth labels.

The new boundary indication at 11f1/1+10.827..11.774 no longer suppresses the
existing right turn at +13.976. In 1186/3, the subsequent manual turn reappears at
18.736 (245 ms later than baseline) and retains its 22.187 end. The opposite
manual request at 11d5/3+46.771 is also retained after a short new left indication.

This does **not** establish zero additional misses or false indications:

- The old 118a/9+57.114 model turn is blocked by the existing isolation after a
  newly admitted manual indication. Recorded lever/lamp activity also exists;
  no retained video establishes a physical miss. Driver/isolation guards were
  not bypassed to force this counterfactual match.
- Five matched holds end earlier. Three encounter recorded lever inputs under
  the new request's tap-echo timing (11e1/7 twice and 11d8/0); two end on boundary
  return (116f/3 and 11f5/7). The largest reductions are 26.516 and 9.151 seconds
  in 11e1/7. Earlier requests shift the echo window while historical lever/lamp
  feedback remains fixed, so physical hold changes cannot be inferred directly.
- Added-request video is retained for 17 of 42 events. Sampled frames show a
  right turn in 11f1/1, but parked-car avoidance in 11d6/0, 11e3/6 and 11f3/9
  is also admitted. These are potential unwanted indications. The other sampled
  scenes are not fully labeled; 25 additions lack matching retained video.

Local isolated checks pass 1,196 tests with nine platform skips; Wiki tests pass
25. Prior-source checks reproduce removed admission/rearm behavior (30 failures,
including assertions about newly introduced state). Native/full CI is recorded
against the published commit. No vehicle writes, CAN actuation or physical lamp
validation were performed. Full before/after tables and private snapshots are in
`.analysis/scratch/2026-11-20-manual-indicator-fix/INDEX.md`.

## November 21: repeated probe delay and a missed consecutive turn

Read-only retrieval added 24 complete rlogs and matching qcamera files from the
two latest drives, 1206 and 1207. All 48 files match the source SHA-256 manifest.
The vehicle ran 3d43de5ca6, whose indicator implementation matches the comparison
baseline 5c970aa516. No device settings, software or actuators were changed.

All 28 observed requests have outgoing SPAS and raw lamp response. Observation
to first SPAS takes approximately 0.43–0.52 seconds; observation to first raw lamp
is 0.503–0.651 seconds, mean 0.576. Each request repeats an unanswered APRK
extended-session probe before the existing 400 ms mode-1 fallback. This is a
controller wait, not evidence that USB transport lost these requests.

Mode 1 now remembers a failed probe for 60 seconds from that probe, only while
stock reception and output ownership remain continuous. New ECU replies, stale
stock, lost ownership, settings changes, expiry or time reversal invalidate it.
Reusing the result does not extend expiry or create a new diagnostic session.
Mode 0 still requires acknowledgement. Panda synchronization, exclusive output
ownership and driver cancellation remain unchanged. An exact before/after
controller-method simulation using the 28 recorded request windows, continuously
healthy stock and no ECU acknowledgement saves 400 ms on 16 requests, averaging
228.6 ms across all 28. These are simulated transmission improvements, not newly
measured physical lamp timings.

A second counterfactual run uses the actual stock CAN frames/CRC progression,
recorded driver validity and direction, model/control freshness and lamp state.
It also transmits all 28 requests and improves the same 16, averaging 229.5 ms
across 28 with recorded scheduling intervals. It assumes continuous process
ownership and unchanged historical ECU responses; it is still not live validation.

In 1207/9, opposite driver steering ends a left turn at 58.104. The following
rightward steering reaches its peak during rearm; the previous detector forgets
its confirmed growth before emission becomes available. Retain that blocked
peak for at most 0.3 seconds and five degrees of unwind, requiring current
same-direction torque and yaw. A new growing movement after admission, released
or opposite torque, a road-curve classification or invalid input clears this
memory. The existing two-second rearm and one continuous dark second remain.
The right request now starts at 1207/10+0.206 and ends at +1.907 on opposite
driver input. Sampled video supports a right turn; the recovered start is still
late relative to steering onset.

The complete retained corpus is now 491 full segments, 62 routes and 550,793
aligned inputs. Immediate-native requests change 434→436, simulated 410 ms
delayed-native 433→435, and external 435→437. Each adds the same two manual-turn
requests, with no removed request and exact start/source/end for every matched
request. The other addition, 118a/10+40.708, lacks retained video; its delayed
native hold reaches the retained-log boundary. Thus these results do not prove
zero false indications or physical lamp completion. Earlier broader candidates
were rejected for suppressing later requests or delaying an existing start by
748 ms; the final bounded-peak rule removes those regressions.

Remaining misses have distinct causes: active automatic lane-change state in
1206/5, narrow/low-confidence modeled lane geometry in 1207/1, and stale or
missing model inputs in 1207/2. Some candidate steering movements are parked-car
avoidance. No broad freshness, lane-width, automatic-steering isolation or
turn-angle gate was removed to force those cases to light.

Local isolated checks pass 1,238 with nine platform skips. Two independent
before/after reproductions confirm the old peak miss and repeated 410 ms wait,
with the latter reduced to 10 ms for a reusable result. A full-controller test
also reproduces and fixes cache cleanup when output is disabled while idle;
the cleanup emits no APRK or SPAS frames. Native/full CI is tied
to the published commit. Private raw evidence and replay outputs are indexed in
`.analysis/scratch/2026-11-21-indicator-fixes/INDEX.md`.
