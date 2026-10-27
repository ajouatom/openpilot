# October 27 turn-indicator drive review

Only carrot-nav-blink. Reviewed joongyu01/carrot-wip-dev at 40aa6abd50f7d6fefa8481d28838596ddad083a8; no upstream CAN changes imported.

## Recorded evidence

Read-only vehicle inventory found the evening and morning drives at October 26 22:44-23:00 and October 27 06:48-07:01 KST. Earlier evening routes in the retained 24-hour inventory were stationary. Downloaded 29 closed full logs, 360,947,687 bytes, with local SHA256 matching independent remote manifest hashes. Both drives used 0071e533a4, AutoBlinkerControl2, AlwaysLateral1, SPAS/test0. Current vehicle is cd80fae432 and actual offroad/noOutput; it had not driven on today's DM/catalog update at the time of this review.

- Evening left indication persisted through the recorded stop around route-relative382.8-388 seconds and ended after turning. A stop does not invariably cancel the existing correction.
- Morning left: cue378.213s, external hold active379.651s, hold cleared391.460s, outgoing0x1EA hold cleared391.507s, CAN-derived lamp off392.138s (approximately06:55:18 KST). Navigation still says turn left,17m ahead; speed3.686km/h. Recorded body heading had changed about78degrees on the preceding bend/turn and steering recentered, prematurely satisfying the completion policy. A later real stalk press is visible398.032s and lamps return for the subsequent turn. This is not proof of a fully stopped vehicle cancelling itself.
- Morning right: cue519.217s, lamp on519.490s, lamp off521.978s (approximately06:57:28), full standstill525.455s. No external physicalBlinkerHold or outgoing0x1EA hold was asserted for this cue. The automatic policy itself remains eligible through the maneuver in ordered input replay. The initial tap did light the lamps, but the completion/ownership acknowledgement needed for sustained hold did not succeed. The old logs do not record the receiver's exact abort reason; do not call this a proven USB, driver-override, counter or timeout fault. No blind retry is added for unconfirmed starts.

CAN-derived lamp states and outgoing queued frames are recorded evidence, not external visual confirmation or proof that each transmitted frame reached an ECU. Raw logs, coordinates, device/session identifiers and analysis scripts remain private.

## Changes and validation

Pass current navigation distance into the existing turn policy. Moving, steering angle, body heading and centering alone cannot finish a turn while navigation still places that same turn ahead. Invalid/stale navigation/pose, opposing turn intent, driver priority and existing expiry remain.

For USB2CAN only, a previously acknowledged and visibly activated ATC turn may request exactly one new one-touch transaction after both lamp states stay off continuously for1.5s. A normal flashing dark interval resets on the next illumination. No retry for an unconfirmed start, manual/Bluetooth/lane-change source, opposing/hazard lamps, missing acknowledgement or cancellation. A new request ID requires a new receiver acknowledgement; original300s total lifetime is retained. The existing receiver still checks unchanged neutral stalk, fresh CAN, guard deadline, one-shot schedules and driver overrides. No CAN format, priority, SPAS behavior, model or Jetson runtime change.

Comma transition diagnostics record sampled source/activation/acceptance/receiver status on changes, at most5Hz, with no session token or request ID. Extra source diagnostics remain local IPC and are excluded from USB wire records. These are sampled diagnostics, not a complete receiver abort trace.

Sorted, timestamp-based open-loop replay of both full drives: morning left request ends391.456s before correction versus404.702s after, preserving the same cue start378.253s. Evening stopped turn end397.963s is unchanged. Morning right policy end541.322s is unchanged: its missing receiver acknowledgement is NOT solved by this replay. Replay does not reproduce live scheduling, receiver feedback, CAN arbitration or physical lamps.

Desktop focused checks:121 passed before the two additional integration cases, then14 recovery cases passed including actual receiver one-touch scheduling/new acknowledgement and sender diagnostic wire exclusion;123 distinct focused cases total. Python lint passes. CI/publication results will be recorded separately. No live actuation, restart, CAN trial or road acceptance was performed for this review.

## Upstream review

The new lane_crossing.py observes model lane geometry, not driver steering torque. With CanfdBlinkerControl enabled, D gear, speed above30km/h (20km/h hysteresis while an APRK transaction is active), and both lamps off, a near line within0.9m and filtered approach rate above0.4m/s plus the opposite line receding above0.4m/s can trigger. Line probability need only have exceeded0.5 within3s; a greater-than0.4m one-frame relabel suppresses detection2s, with5s cooldown. Engaged/latActive/steeringPressed are not required, so this cannot distinguish deliberate manual lane changes from drift or some geometry errors. The contributor reports35/38 detections with7 false triggers in the main drive, and2/2 with3 false in a holdout; these are contributor results, not independent acceptance.

Output still requests APRK extended session, waits five100Hz controller frames, sends communication-disable without checking ECU replies, and injects SPAS1/2 at100Hz for the roughly1s pulse. A default-session request follows; restored OEM behavior is not acknowledged. Source explicitly notes OEM50Hz SPAS may remain and interleave. Stock SPAS2 fields are copied for GV80 support; this does not eliminate arbitration/counter conflict or establish Ioniq5PE acceptance. No model freshness/validity or complete right-line/probability array checks exist in the new detector. Adopt neither the actuation nor unconditional crossing trigger. A future suggestion-only detector could be studied separately using this vehicle's logs and fresh finite geometry, without modifying steering or injecting SPAS.

## Publication result

Source9779819859 is published only on carrot-nav-blink. CI40430803117 passes Windows installer and Linux:232 monitoring,62 codec,94 updater,1,648 broad/56skips,119 navigation,11 storage and11 network checks. Final123 focused desktop tests, lint and mapped user-doc validation pass; counts overlap CI. The recovery receiver integration uses the real Controller policy with simulated time/CAN, not a physical CAN device.

Vehicle installation/restart was not requested or performed in this review; current vehicle remains cd80fae432. Normal Git Pull and comma restart activates this comma-side change; no new Jetson bundle, Jetson reboot or model download is needed. A read-only attempt to retrieve Jetson journal over its LAN address timed out. The receiver abort cause remains unresolved and physical recovery still needs acceptance.

Panda summary logs show no error-counter increase at either morning symptom. A large pre-existing canState0 total10004410/TXlost1 is already present at route start and remains constant; it must not be attributed to these blinker events. canState1 gains one error at route shutdown. The Jetson USB2CAN adapter's own historical counters were not available, so this is not an all-buses-error-free claim.
